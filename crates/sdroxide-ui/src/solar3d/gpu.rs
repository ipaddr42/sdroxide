//! The wgpu side of the solar-system view.
//!
//! The whole 3D scene is rendered **offscreen inside
//! [`CallbackTrait::prepare`]**, into colour, depth and MSAA targets this module
//! owns; the only thing that runs inside egui's own render pass is a three-vertex
//! blit of the result.
//!
//! The alternative — turning on `NativeOptions::depth_buffer` — would attach a
//! depth target to *every* egui pass in the process, which in turn forces a
//! matching `depth_stencil` state onto the waterfall pipelines and onto the
//! WebGL2 browser path. Rendering apart costs one full-screen texture read and
//! buys complete independence: `src/gui_main.rs` needs no changes at all.

use eframe::egui_wgpu::{CallbackResources, CallbackTrait, RenderState, ScreenDescriptor, wgpu};

use crate::basemap::{BORDER_PNG, CITY_PNG, LAND_PNG, RIVER_PNG};

use super::mesh;
use super::scene::{DrawData, Flashes, Globals, LineInst, Prim, Scene, SpriteInst};

/// Size every body map is stored at. One size for all of them, because they
/// live in a single array texture.
const BODY_MAP_W: u32 = 2048;
const BODY_MAP_H: u32 = 1024;

const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;
/// Reversed-Z: the far plane is 0, so that is what depth clears to.
const DEPTH_CLEAR: f32 = 0.0;
/// Offscreen targets are rounded up to this, so dragging a window edge
/// reallocates a handful of times instead of sixty times a second.
const ALLOC_STEP: u32 = 128;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct BlitUniform {
    uv_scale: [f32; 4],
}

/// Colour/depth/MSAA targets, reallocated only when the window grows past the
/// current allocation (or shrinks well below it).
struct Targets {
    alloc: (u32, u32),
    color: wgpu::TextureView,
    msaa: Option<wgpu::TextureView>,
    depth: wgpu::TextureView,
    blit_bg: wgpu::BindGroup,
    // Views borrow from these; keeping them alive keeps the views valid.
    _color_tex: wgpu::Texture,
    _msaa_tex: Option<wgpu::Texture>,
    _depth_tex: wgpu::Texture,
}

pub struct SolarResources {
    body_pipe: wgpu::RenderPipeline,
    aurora_pipe: wgpu::RenderPipeline,
    cloud_pipe: wgpu::RenderPipeline,
    prop_pipe: wgpu::RenderPipeline,
    cloud_march_pipe: wgpu::RenderPipeline,
    cone_pipe: wgpu::RenderPipeline,
    ring_pipe: wgpu::RenderPipeline,
    tail_pipe: wgpu::RenderPipeline,
    line_pipe: wgpu::RenderPipeline,
    sprite_pipe: wgpu::RenderPipeline,
    blit_pipe: wgpu::RenderPipeline,

    scene_bgl: wgpu::BindGroupLayout,
    draw_bgl: wgpu::BindGroupLayout,
    blit_bgl: wgpu::BindGroupLayout,
    scene_bg: wgpu::BindGroup,
    draw_bg: wgpu::BindGroup,

    globals_buf: wgpu::Buffer,
    draw_buf: wgpu::Buffer,
    draw_stride: u32,
    draw_cap: u32,
    blit_uniform: wgpu::Buffer,

    sphere_vb: wgpu::Buffer,
    sphere_ib: wgpu::Buffer,
    sphere_indices: u32,
    cone_vb: wgpu::Buffer,
    cone_ib: wgpu::Buffer,
    cone_indices: u32,
    ring_vb: wgpu::Buffer,
    ring_ib: wgpu::Buffer,
    ring_indices: u32,
    plume_vb: wgpu::Buffer,
    plume_ib: wgpu::Buffer,
    plume_indices: u32,
    quad_vb: wgpu::Buffer,

    line_buf: wgpu::Buffer,
    line_cap: usize,
    sprite_buf: wgpu::Buffer,
    sprite_cap: usize,
    star_buf: wgpu::Buffer,
    star_count: u32,

    land_view: wgpu::TextureView,
    border_view: wgpu::TextureView,
    river_view: wgpu::TextureView,
    city_view: wgpu::TextureView,
    body_map_view: wgpu::TextureView,
    sun_tex: wgpu::Texture,
    sun_view: wgpu::TextureView,
    /// Bumped whenever a new solar image is uploaded, so the scene bind group
    /// is rebuilt exactly once per image rather than every frame.
    sun_gen: u64,
    aurora_tex: wgpu::Texture,
    aurora_view: wgpu::TextureView,
    cloud_tex: wgpu::Texture,
    cloud_view: wgpu::TextureView,
    prop_tex: wgpu::Texture,
    prop_view: wgpu::TextureView,
    flash_buf: wgpu::Buffer,
    /// As `sun_gen`, for the OVATION grid. Its size is fixed, so a new grid is
    /// a texture write and never a bind-group rebuild.
    aurora_gen: u64,
    cloud_gen: u64,
    prop_gen: u64,
    sampler: wgpu::Sampler,

    sample_count: u32,
    /// Colour format of the offscreen targets. Must equal egui's surface
    /// format so the blit is a straight copy with no conversion.
    blit_format: wgpu::TextureFormat,
    targets: Option<Targets>,
    max_dim: u32,
}

/// Build the pipelines and static meshes. Idempotent: `callback_resources` is a
/// process-wide type map shared by every viewport, so closing and reopening the
/// window must not rebuild (or leak) any of this.
pub fn init(rs: &RenderState) {
    if rs.renderer.read().callback_resources.get::<SolarResources>().is_some() {
        return;
    }
    let res = build(rs);
    rs.renderer.write().callback_resources.insert(res);
}

fn build(rs: &RenderState) -> SolarResources {
    use wgpu::util::DeviceExt as _;
    let device = &rs.device;
    let limits = device.limits();

    // 4× MSAA where the surface format supports it: untextured spheres and
    // two-pixel orbit lines alias badly, and this pass is ours alone so the
    // cost lands nowhere else.
    let feats = rs.adapter.get_texture_format_features(rs.target_format).flags;
    let sample_count = if feats.contains(
        wgpu::TextureFormatFeatureFlags::MULTISAMPLE_X4
            | wgpu::TextureFormatFeatureFlags::MULTISAMPLE_RESOLVE,
    ) {
        4
    } else {
        1
    };

    let shader = |label: &str, src: &str| {
        device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some(label),
            source: wgpu::ShaderSource::Wgsl(src.into()),
        })
    };
    let body_sh = shader("solar-body", include_str!("../shaders/solar_body.wgsl"));
    let aurora_sh = shader("solar-aurora", include_str!("../shaders/solar_aurora.wgsl"));
    let cloud_sh = shader("solar-cloud", include_str!("../shaders/solar_cloud.wgsl"));
    let prop_sh = shader("solar-prop", include_str!("../shaders/solar_prop.wgsl"));
    let cloud_march_sh =
        shader("solar-cloud-march", include_str!("../shaders/solar_cloud_march.wgsl"));
    let cone_sh = shader("solar-cone", include_str!("../shaders/solar_cone.wgsl"));
    let ring_sh = shader("solar-ring", include_str!("../shaders/solar_ring.wgsl"));
    let tail_sh = shader("solar-tail", include_str!("../shaders/solar_tail.wgsl"));
    let line_sh = shader("solar-line", include_str!("../shaders/solar_line.wgsl"));
    let sprite_sh = shader("solar-sprite", include_str!("../shaders/solar_sprite.wgsl"));
    let blit_sh = shader("solar-blit", include_str!("../shaders/solar_blit.wgsl"));

    let uniform_entry = |binding: u32, dynamic: bool, size: u64| wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Uniform,
            has_dynamic_offset: dynamic,
            min_binding_size: wgpu::BufferSize::new(size),
        },
        count: None,
    };
    let tex_entry = |binding: u32| wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::FRAGMENT,
        ty: wgpu::BindingType::Texture {
            sample_type: wgpu::TextureSampleType::Float { filterable: true },
            view_dimension: wgpu::TextureViewDimension::D2,
            multisampled: false,
        },
        count: None,
    };

    let scene_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("solar-scene-bgl"),
        entries: &[
            uniform_entry(0, false, std::mem::size_of::<Globals>() as u64),
            tex_entry(1),
            tex_entry(2),
            wgpu::BindGroupLayoutEntry {
                binding: 3,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                count: None,
            },
            tex_entry(4),
            tex_entry(5),
            wgpu::BindGroupLayoutEntry {
                binding: 6,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    view_dimension: wgpu::TextureViewDimension::D2Array,
                    multisampled: false,
                },
                count: None,
            },
            tex_entry(7),
            uniform_entry(8, false, std::mem::size_of::<Flashes>() as u64),
            tex_entry(9),
            tex_entry(10),
            tex_entry(11),
        ],
    });
    let draw_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("solar-draw-bgl"),
        entries: &[uniform_entry(0, true, std::mem::size_of::<DrawData>() as u64)],
    });
    let blit_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("solar-blit-bgl"),
        entries: &[
            tex_entry(0),
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                count: None,
            },
            uniform_entry(2, false, std::mem::size_of::<BlitUniform>() as u64),
        ],
    });

    let scene_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("solar-scene-layout"),
        bind_group_layouts: &[Some(&scene_bgl)],
        immediate_size: 0,
    });
    let draw_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("solar-draw-layout"),
        bind_group_layouts: &[Some(&scene_bgl), Some(&draw_bgl)],
        immediate_size: 0,
    });
    let blit_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("solar-blit-layout"),
        bind_group_layouts: &[Some(&blit_bgl)],
        immediate_size: 0,
    });

    let mesh_layout = wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<mesh::Vertex>() as u64,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x2],
    };
    let quad_layout = wgpu::VertexBufferLayout {
        array_stride: 8,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &wgpu::vertex_attr_array![0 => Float32x2],
    };
    // LineInst and SpriteInst share a stride and the same attribute offsets;
    // only the meaning of slots 2..4 differs, which the shaders resolve.
    let inst_layout = wgpu::VertexBufferLayout {
        array_stride: 48,
        step_mode: wgpu::VertexStepMode::Instance,
        attributes: &wgpu::vertex_attr_array![
            1 => Float32x3, 2 => Float32, 3 => Float32x3, 4 => Float32x4
        ],
    };
    let sprite_inst_layout = wgpu::VertexBufferLayout {
        array_stride: 48,
        step_mode: wgpu::VertexStepMode::Instance,
        attributes: &wgpu::vertex_attr_array![
            1 => Float32x3, 2 => Float32, 3 => Float32x4, 4 => Float32x4
        ],
    };

    // Premultiplied "over". Additive strokes, glows and the aurora emit zero
    // alpha so they add light without occluding; markers emit real alpha and
    // sit on top.
    let premultiplied = wgpu::BlendState {
        color: wgpu::BlendComponent {
            src_factor: wgpu::BlendFactor::One,
            dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
            operation: wgpu::BlendOperation::Add,
        },
        alpha: wgpu::BlendComponent {
            src_factor: wgpu::BlendFactor::One,
            dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
            operation: wgpu::BlendOperation::Add,
        },
    };

    let depth_state = |write: bool, compare: wgpu::CompareFunction| {
        Some(wgpu::DepthStencilState {
            format: DEPTH_FORMAT,
            depth_write_enabled: Some(write),
            depth_compare: Some(compare),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        })
    };

    let make_pipe = |label: &str,
                     layout: &wgpu::PipelineLayout,
                     sh: &wgpu::ShaderModule,
                     buffers: &[wgpu::VertexBufferLayout],
                     blend: Option<wgpu::BlendState>,
                     depth: Option<wgpu::DepthStencilState>,
                     samples: u32| {
        // wgpu 30 made vertex buffer slots individually optional, so a pipeline
        // can leave a gap in the slot numbering. Nothing here does — every slot
        // this file declares is bound — so the layouts are wrapped once at the
        // boundary rather than each call site carrying a `Some`.
        let buffers: Vec<Option<wgpu::VertexBufferLayout>> =
            buffers.iter().cloned().map(Some).collect();
        device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some(label),
            layout: Some(layout),
            vertex: wgpu::VertexState {
                module: sh,
                entry_point: Some("vs"),
                compilation_options: Default::default(),
                buffers: &buffers,
            },
            fragment: Some(wgpu::FragmentState {
                module: sh,
                entry_point: Some("fs"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: rs.target_format,
                    blend,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                // No culling: the sphere and cone windings are irrelevant with
                // a depth buffer, and the cone must show its inside surface.
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: depth,
            multisample: wgpu::MultisampleState { count: samples, ..Default::default() },
            multiview_mask: None,
            cache: None,
        })
    };

    let body_pipe = make_pipe(
        "solar-body",
        &draw_layout,
        &body_sh,
        &[mesh_layout.clone()],
        None,
        depth_state(true, wgpu::CompareFunction::Greater),
        sample_count,
    );
    // The aurora shells are emission: additive, and no depth write, so twenty
    // of them stack into one glow. They still *test* depth, which is what puts
    // the far side of the oval behind the planet for free.
    let aurora_pipe = make_pipe(
        "solar-aurora",
        &draw_layout,
        &aurora_sh,
        &[mesh_layout.clone()],
        Some(premultiplied),
        depth_state(false, wgpu::CompareFunction::Greater),
        sample_count,
    );
    // Cloud occludes: a deck hides the coastline under it, so unlike the aurora
    // these emit real alpha and composite rather than only adding. Still no
    // depth write, so the shells blend into one deck — and so the far side of
    // each that clears the planet's silhouette is drawn too, which is the limb,
    // which is the one place an eighteen-kilometre slab is more than a pixel
    // thick.
    let cloud_pipe = make_pipe(
        "solar-cloud",
        &draw_layout,
        &cloud_sh,
        &[mesh_layout.clone()],
        Some(premultiplied),
        depth_state(false, wgpu::CompareFunction::Greater),
        sample_count,
    );
    // Paint on the surface, not light standing above it: composited over, so
    // the coastline shows through a heat cell rather than being added to. No
    // depth write, so the layer never occludes the markers on top of it, but it
    // still *tests* depth, which is what keeps the far side of the globe behind
    // the near side.
    let prop_pipe = make_pipe(
        "solar-prop",
        &draw_layout,
        &prop_sh,
        &[mesh_layout.clone()],
        Some(premultiplied),
        depth_state(false, wgpu::CompareFunction::Greater),
        sample_count,
    );
    let cloud_march_pipe = make_pipe(
        "solar-cloud-march",
        &draw_layout,
        &cloud_march_sh,
        &[mesh_layout.clone()],
        Some(premultiplied),
        depth_state(false, wgpu::CompareFunction::Greater),
        sample_count,
    );
    let cone_pipe = make_pipe(
        "solar-cone",
        &draw_layout,
        &cone_sh,
        &[mesh_layout.clone()],
        Some(premultiplied),
        // No depth write, so overlapping cones blend rather than clip.
        depth_state(false, wgpu::CompareFunction::Greater),
        sample_count,
    );
    // Rings are a transparent sheet: they blend over whatever is behind them
    // and must not write depth, or the near half would hide the far half.
    let ring_pipe = make_pipe(
        "solar-ring",
        &draw_layout,
        &ring_sh,
        &[mesh_layout.clone()],
        Some(premultiplied),
        depth_state(false, wgpu::CompareFunction::Greater),
        sample_count,
    );
    // A comet tail is emission through emission: additive, and no depth write,
    // so the dust tail and the ion tail sum where they overlap instead of one
    // clipping the other. It still *tests* depth, which is what puts the far
    // half of a tail behind whatever it is streaming past.
    let tail_pipe = make_pipe(
        "solar-tail",
        &draw_layout,
        &tail_sh,
        &[mesh_layout.clone()],
        Some(premultiplied),
        depth_state(false, wgpu::CompareFunction::Greater),
        sample_count,
    );
    let line_pipe = make_pipe(
        "solar-line",
        &scene_layout,
        &line_sh,
        &[quad_layout.clone(), inst_layout],
        Some(premultiplied),
        depth_state(false, wgpu::CompareFunction::GreaterEqual),
        sample_count,
    );
    let sprite_pipe = make_pipe(
        "solar-sprite",
        &scene_layout,
        &sprite_sh,
        &[quad_layout, sprite_inst_layout],
        Some(premultiplied),
        // GreaterEqual, not Greater: the star field sits on the far plane,
        // which equals the depth clear value.
        depth_state(false, wgpu::CompareFunction::GreaterEqual),
        sample_count,
    );
    // The blit runs inside egui's pass, which has neither depth nor MSAA.
    let blit_pipe = make_pipe("solar-blit", &blit_layout, &blit_sh, &[], None, None, 1);

    // ── Static meshes ───────────────────────────────────────────────────────
    let (sv, si) = mesh::sphere();
    let (cv, ci) = mesh::cone();
    let (rv, ri) = mesh::ring();
    let (pv, pi) = mesh::plume();
    let quad = mesh::quad();
    let vb = |label: &str, data: &[u8], usage: wgpu::BufferUsages| {
        device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(label),
            contents: data,
            usage,
        })
    };
    let sphere_vb = vb("solar-sphere-vb", bytemuck::cast_slice(&sv), wgpu::BufferUsages::VERTEX);
    let sphere_ib = vb("solar-sphere-ib", bytemuck::cast_slice(&si), wgpu::BufferUsages::INDEX);
    let cone_vb = vb("solar-cone-vb", bytemuck::cast_slice(&cv), wgpu::BufferUsages::VERTEX);
    let cone_ib = vb("solar-cone-ib", bytemuck::cast_slice(&ci), wgpu::BufferUsages::INDEX);
    let ring_vb = vb("solar-ring-vb", bytemuck::cast_slice(&rv), wgpu::BufferUsages::VERTEX);
    let ring_ib = vb("solar-ring-ib", bytemuck::cast_slice(&ri), wgpu::BufferUsages::INDEX);
    let plume_vb = vb("solar-plume-vb", bytemuck::cast_slice(&pv), wgpu::BufferUsages::VERTEX);
    let plume_ib = vb("solar-plume-ib", bytemuck::cast_slice(&pi), wgpu::BufferUsages::INDEX);
    let quad_vb = vb("solar-quad-vb", bytemuck::cast_slice(&quad), wgpu::BufferUsages::VERTEX);

    let stars = super::scene::stars();
    let star_buf = vb("solar-stars", bytemuck::cast_slice(&stars), wgpu::BufferUsages::VERTEX);

    // ── Uniform and instance buffers ────────────────────────────────────────
    let globals_buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("solar-globals"),
        size: std::mem::size_of::<Globals>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let draw_stride =
        limits.min_uniform_buffer_offset_alignment.max(std::mem::size_of::<DrawData>() as u32);
    let draw_cap = 64u32;
    let draw_buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("solar-draws"),
        size: (draw_stride * draw_cap) as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let blit_uniform = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("solar-blit-uniform"),
        size: std::mem::size_of::<BlitUniform>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let inst_buf = |label: &str, cap: usize| {
        device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: (cap * 48) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        })
    };
    let line_cap = 1024;
    let sprite_cap = 256;

    // ── Textures ────────────────────────────────────────────────────────────
    let map_dim = MAX_MAP_DIM.min(limits.max_texture_dimension_2d);
    // Coastline and borders are the map; rivers and the urban areas the night
    // side is lit by are detail on it, and give first when the budget is tight.
    let land = upload_map(device, &rs.queue, "solar-land-mask", LAND_PNG, map_dim);
    let border_view = upload_map(device, &rs.queue, "solar-borders", BORDER_PNG, map_dim);
    let detail_dim = MAX_DETAIL_DIM.min(limits.max_texture_dimension_2d);
    let river_view = upload_map(device, &rs.queue, "solar-rivers", RIVER_PNG, detail_dim);
    let city_view = upload_map(device, &rs.queue, "solar-cities", CITY_PNG, detail_dim);
    let body_maps = upload_body_maps(device, &rs.queue);
    let sun_tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("solar-sun-tex"),
        size: wgpu::Extent3d { width: 1, height: 1, depth_or_array_layers: 1 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    rs.queue.write_texture(
        wgpu::TexelCopyTextureInfo {
            texture: &sun_tex,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        &[0u8, 0, 0, 255],
        wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(4), rows_per_image: Some(1) },
        wgpu::Extent3d { width: 1, height: 1, depth_or_array_layers: 1 },
    );
    let sun_view = sun_tex.create_view(&Default::default());

    // The OVATION grid, one texel per degree. Created empty and never resized —
    // a new issue is a texture write, not a reallocation — and zero everywhere
    // means "no aurora", which is exactly what should be drawn before the first
    // one arrives.
    let aurora_tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("solar-aurora-tex"),
        size: wgpu::Extent3d {
            width: sdroxide_solar::aurora::GRID_W as u32,
            height: sdroxide_solar::aurora::GRID_H as u32,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::R8Unorm,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    let aurora_view = aurora_tex.create_view(&Default::default());

    // The propagation heat, already resolved to colours by `prop_map`. Straight
    // RGBA rather than a per-band array plus lookup tables: the flat map in the
    // operating panel uploads these same bytes to egui, and one texture is the
    // only way to be certain the two views cannot disagree. Created empty,
    // which is transparent, which is what should be drawn before any traffic
    // has been heard.
    let prop_tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("solar-prop-tex"),
        size: wgpu::Extent3d {
            width: sdroxide_types::PROP_GRID_W as u32,
            height: sdroxide_types::PROP_GRID_H as u32,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8Unorm,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    let prop_view = prop_tex.create_view(&Default::default());

    // The cloud field: red is optical thickness, green is cloud-top height.
    // Created empty for the same reason as the aurora grid — zero is "no
    // cloud", which is the right thing to draw before the first mosaic lands —
    // and never resized, so a new mosaic is a write and not a reallocation. A
    // row is 2048 bytes, which is already the 256-byte multiple `write_texture`
    // insists on, so unlike the 360-wide aurora grid this needs no padding.
    let cloud_tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("solar-cloud-tex"),
        size: wgpu::Extent3d {
            width: sdroxide_solar::clouds::GRID_W as u32,
            height: sdroxide_solar::clouds::GRID_H as u32,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rg8Unorm,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    let cloud_view = cloud_tex.create_view(&Default::default());
    let flash_buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("solar-flashes"),
        size: std::mem::size_of::<Flashes>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("solar-sampler"),
        address_mode_u: wgpu::AddressMode::Repeat,
        address_mode_v: wgpu::AddressMode::ClampToEdge,
        address_mode_w: wgpu::AddressMode::ClampToEdge,
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        mipmap_filter: wgpu::MipmapFilterMode::Linear,
        ..Default::default()
    });

    let scene_bg = make_scene_bg(
        device,
        &scene_bgl,
        &globals_buf,
        &land,
        &sun_view,
        &aurora_view,
        &border_view,
        &river_view,
        &city_view,
        &body_maps,
        &cloud_view,
        &prop_view,
        &flash_buf,
        &sampler,
    );
    let draw_bg = make_draw_bg(device, &draw_bgl, &draw_buf);

    SolarResources {
        body_pipe,
        aurora_pipe,
        cloud_pipe,
        prop_pipe,
        cloud_march_pipe,
        cone_pipe,
        ring_pipe,
        tail_pipe,
        line_pipe,
        sprite_pipe,
        blit_pipe,
        scene_bgl,
        draw_bgl,
        blit_bgl,
        scene_bg,
        draw_bg,
        globals_buf,
        draw_buf,
        draw_stride,
        draw_cap,
        blit_uniform,
        sphere_vb,
        sphere_ib,
        sphere_indices: si.len() as u32,
        cone_vb,
        cone_ib,
        cone_indices: ci.len() as u32,
        ring_vb,
        ring_ib,
        ring_indices: ri.len() as u32,
        plume_vb,
        plume_ib,
        plume_indices: pi.len() as u32,
        quad_vb,
        line_buf: inst_buf("solar-lines", line_cap),
        line_cap,
        sprite_buf: inst_buf("solar-sprites", sprite_cap),
        sprite_cap,
        star_buf,
        star_count: stars.len() as u32,
        land_view: land,
        border_view,
        river_view,
        city_view,
        body_map_view: body_maps,
        sun_tex,
        sun_view,
        sun_gen: 0,
        aurora_tex,
        aurora_view,
        aurora_gen: 0,
        cloud_tex,
        cloud_view,
        prop_tex,
        prop_view,
        flash_buf,
        cloud_gen: 0,
        prop_gen: 0,
        sampler,
        sample_count,
        blit_format: rs.target_format,
        targets: None,
        max_dim: limits.max_texture_dimension_2d,
    }
}

#[allow(clippy::too_many_arguments)]
fn make_scene_bg(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    globals: &wgpu::Buffer,
    land: &wgpu::TextureView,
    sun: &wgpu::TextureView,
    aurora: &wgpu::TextureView,
    borders: &wgpu::TextureView,
    rivers: &wgpu::TextureView,
    cities: &wgpu::TextureView,
    body_maps: &wgpu::TextureView,
    clouds: &wgpu::TextureView,
    prop: &wgpu::TextureView,
    flashes: &wgpu::Buffer,
    sampler: &wgpu::Sampler,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("solar-scene-bg"),
        layout,
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: globals.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::TextureView(land) },
            wgpu::BindGroupEntry { binding: 2, resource: wgpu::BindingResource::TextureView(sun) },
            wgpu::BindGroupEntry { binding: 3, resource: wgpu::BindingResource::Sampler(sampler) },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: wgpu::BindingResource::TextureView(aurora),
            },
            wgpu::BindGroupEntry {
                binding: 5,
                resource: wgpu::BindingResource::TextureView(borders),
            },
            wgpu::BindGroupEntry {
                binding: 6,
                resource: wgpu::BindingResource::TextureView(body_maps),
            },
            wgpu::BindGroupEntry {
                binding: 7,
                resource: wgpu::BindingResource::TextureView(clouds),
            },
            wgpu::BindGroupEntry { binding: 8, resource: flashes.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 9, resource: wgpu::BindingResource::TextureView(prop) },
            wgpu::BindGroupEntry {
                binding: 10,
                resource: wgpu::BindingResource::TextureView(rivers),
            },
            wgpu::BindGroupEntry {
                binding: 11,
                resource: wgpu::BindingResource::TextureView(cities),
            },
        ],
    })
}

fn make_draw_bg(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    buf: &wgpu::Buffer,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("solar-draw-bg"),
        layout,
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                buffer: buf,
                offset: 0,
                size: wgpu::BufferSize::new(std::mem::size_of::<DrawData>() as u64),
            }),
        }],
    })
}

/// The bodies drawn from real imagery rather than procedurally, in the layer
/// order the shader's `STYLE`/`MAP_*` constants use. All 2048×1024
/// equirectangular and public domain; see `assets/bodies/README.md` for the
/// provenance of each.
///
/// These four are the ones a viewer can *check*. Everybody has seen the Moon,
/// Mars's dark markings and caps are what a small telescope shows, Jupiter's
/// belts and the Great Red Spot are on every poster, and Saturn is exactly as
/// bland as it looks here — no amount of noise-and-ellipses puts Imbrium, Syrtis
/// Major or the GRS where the eye expects them.
const BODY_MAPS: [(&str, &[u8]); 4] = [
    ("moon", include_bytes!("../../assets/bodies/moon.jpg")),
    ("mars", include_bytes!("../../assets/bodies/mars.jpg")),
    ("jupiter", include_bytes!("../../assets/bodies/jupiter.jpg")),
    ("saturn", include_bytes!("../../assets/bodies/saturn.jpg")),
];

/// Largest edge the globe's masks are uploaded at.
///
/// Native keeps the land asset's full 8192: the camera can fly right down to
/// the surface, which is what that resolution is for, and 45 MB of R8 with its
/// mip chain is a fair price for a desktop GPU. The browser's cap puts land two
/// levels down at 2048×1024 — 45 MB is a lot to hand a tab, building that chain
/// is tens of milliseconds on the page's only thread, and a globe that is
/// rarely more than a panel wide there gains nothing from the rest. The pair
/// still costs a browser about 6 MB, which is what it cost before the land
/// asset grew. (The PNG decode itself is unchanged — that is fixed by the
/// asset, and one asset shared by both targets is worth more than the saving.)
#[cfg(not(target_arch = "wasm32"))]
const MAX_MAP_DIM: u32 = 8192;
#[cfg(target_arch = "wasm32")]
const MAX_MAP_DIM: u32 = 2160;

/// The same, for the rivers and the city lights.
///
/// Natively they are structure like everything else and go up whole. In the
/// browser they are the first thing to give: they are detail *on* a globe that
/// is rarely more than a panel wide there, and at their own 4320 the pair would
/// cost a tab as much again as the coastline and the borders together.
#[cfg(not(target_arch = "wasm32"))]
const MAX_DETAIL_DIM: u32 = 4320;
#[cfg(target_arch = "wasm32")]
const MAX_DETAIL_DIM: u32 = 1080;

/// Decode one of those maps into an R8 texture with a full mip chain, with the
/// base level no larger than `max_dim`.
///
/// The mips are not optional at this resolution: 8192 texels of coastline
/// crammed into a 40-pixel globe without them is a shimmering mess, and the
/// borders — one texel wide — would strobe in and out as the camera moves.
/// With them, both fade out gracefully as the Earth shrinks.
fn upload_map(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    label: &str,
    png: &[u8],
    max_dim: u32,
) -> wgpu::TextureView {
    // A checked-in asset failing to decode is not a reason to lose the window;
    // it costs the layer instead, and says so.
    let gray = match image::load_from_memory_with_format(png, image::ImageFormat::Png) {
        Ok(img) => img.to_luma8(),
        Err(e) => {
            eprintln!("sdroxide: decoding {label}: {e}");
            image::GrayImage::from_raw(1, 1, vec![0u8]).expect("1×1")
        }
    };
    let (w, h) = gray.dimensions();
    let mut levels: Vec<(u32, u32, Vec<u8>)> = vec![(w, h, gray.into_raw())];
    while levels.last().is_some_and(|(w, h, _)| *w > 1 || *h > 1) {
        levels.push(halve(levels.last().expect("just checked")));
    }

    // Start the chain at the first level that fits. Creating a texture wider
    // than the device allows is a validation failure, not a soft degradation,
    // and `Limits::downlevel_webgl2_defaults` caps 2D textures at 2048 — far
    // below these assets. egui-wgpu asks for 8192 even on the GL backend, so in
    // practice this only bites where the budget above is lower or a device
    // grants less than was asked; either way, losing detail beats losing the
    // globe.
    let base = levels
        .iter()
        .position(|(w, h, _)| *w <= max_dim && *h <= max_dim)
        .unwrap_or(levels.len() - 1);
    let levels = &levels[base..];
    let (w, h, _) = levels[0];

    let tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some(label),
        size: wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
        mip_level_count: levels.len() as u32,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::R8Unorm,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    for (level, (lw, lh, data)) in levels.iter().enumerate() {
        // `write_texture` wants rows aligned to 256 bytes, which no level here
        // happens to be.
        let stride = lw.div_ceil(256) * 256;
        let mut padded = vec![0u8; (stride * lh) as usize];
        for row in 0..*lh {
            let (src, dst) = ((row * lw) as usize, (row * stride) as usize);
            padded[dst..dst + *lw as usize].copy_from_slice(&data[src..src + *lw as usize]);
        }
        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &tex,
                mip_level: level as u32,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            &padded,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(stride),
                rows_per_image: Some(*lh),
            },
            wgpu::Extent3d { width: *lw, height: *lh, depth_or_array_layers: 1 },
        );
    }
    tex.create_view(&Default::default())
}

/// The body maps, as one array texture with a mip chain: one layer per body,
/// indexed by the draw's style, so adding a map costs a JPEG and a constant
/// rather than another binding.
fn upload_body_maps(device: &wgpu::Device, queue: &wgpu::Queue) -> wgpu::TextureView {
    let decoded: Vec<image::RgbaImage> = BODY_MAPS
        .iter()
        .map(|(name, bytes)| {
            match image::load_from_memory_with_format(bytes, image::ImageFormat::Jpeg) {
                Ok(img) => img.to_rgba8(),
                Err(e) => {
                    // A checked-in asset failing to decode should cost that
                    // body's surface, not the window.
                    eprintln!("sdroxide: decoding the {name} map: {e}");
                    image::RgbaImage::from_pixel(
                        BODY_MAP_W,
                        BODY_MAP_H,
                        image::Rgba([120, 116, 112, 255]),
                    )
                }
            }
        })
        .collect();

    let mips = (BODY_MAP_W.max(BODY_MAP_H).ilog2() + 1) as usize;
    let tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("solar-body-maps"),
        size: wgpu::Extent3d {
            width: BODY_MAP_W,
            height: BODY_MAP_H,
            depth_or_array_layers: decoded.len() as u32,
        },
        mip_level_count: mips as u32,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        // sRGB: these are photographs, and the offscreen target is encoded, so
        // the hardware has to linearise them on the way in.
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });

    for (layer, img) in decoded.into_iter().enumerate() {
        // Every map is the same size, so a mismatched asset is dropped rather
        // than smeared across the wrong latitudes.
        if img.dimensions() != (BODY_MAP_W, BODY_MAP_H) {
            eprintln!(
                "sdroxide: {} is {:?}, expected {BODY_MAP_W}×{BODY_MAP_H}",
                BODY_MAPS[layer].0,
                img.dimensions()
            );
            continue;
        }
        let mut level = (BODY_MAP_W, BODY_MAP_H, img.into_raw());
        for mip in 0..mips {
            queue.write_texture(
                wgpu::TexelCopyTextureInfo {
                    texture: &tex,
                    mip_level: mip as u32,
                    origin: wgpu::Origin3d { x: 0, y: 0, z: layer as u32 },
                    aspect: wgpu::TextureAspect::All,
                },
                &level.2,
                wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    // Four bytes per texel, so a power-of-two width is already
                    // past the 256-byte row alignment `write_texture` wants.
                    bytes_per_row: Some(level.0 * 4),
                    rows_per_image: Some(level.1),
                },
                wgpu::Extent3d { width: level.0, height: level.1, depth_or_array_layers: 1 },
            );
            if mip + 1 < mips {
                level = halve_rgba(&level);
            }
        }
    }
    tex.create_view(&wgpu::TextureViewDescriptor {
        dimension: Some(wgpu::TextureViewDimension::D2Array),
        ..Default::default()
    })
}

/// [`halve`] for four-channel data.
fn halve_rgba((w, h, src): &(u32, u32, Vec<u8>)) -> (u32, u32, Vec<u8>) {
    let (nw, nh) = ((w / 2).max(1), (h / 2).max(1));
    let mut out = vec![0u8; (nw * nh * 4) as usize];
    for y in 0..nh {
        for x in 0..nw {
            let (x0, y0) = (x * 2, y * 2);
            let (x1, y1) = ((x0 + 1).min(w - 1), (y0 + 1).min(h - 1));
            for c in 0..4 {
                let at = |x: u32, y: u32| src[((y * w + x) * 4 + c) as usize] as u32;
                let sum = at(x0, y0) + at(x1, y0) + at(x0, y1) + at(x1, y1);
                out[((y * nw + x) * 4 + c) as usize] = ((sum + 2) / 4) as u8;
            }
        }
    }
    (nw, nh, out)
}

/// One mip level down: a 2×2 box filter, with the odd row/column carried over
/// rather than dropped.
fn halve((w, h, src): &(u32, u32, Vec<u8>)) -> (u32, u32, Vec<u8>) {
    let (nw, nh) = ((w / 2).max(1), (h / 2).max(1));
    let mut out = vec![0u8; (nw * nh) as usize];
    for y in 0..nh {
        for x in 0..nw {
            let (x0, y0) = (x * 2, y * 2);
            let (x1, y1) = ((x0 + 1).min(w - 1), (y0 + 1).min(h - 1));
            let at = |x: u32, y: u32| src[(y * w + x) as usize] as u32;
            let sum = at(x0, y0) + at(x1, y0) + at(x0, y1) + at(x1, y1);
            out[(y * nw + x) as usize] = ((sum + 2) / 4) as u8;
        }
    }
    (nw, nh, out)
}

impl SolarResources {
    /// Allocate (or keep) offscreen targets big enough for `size`.
    fn ensure_targets(&mut self, device: &wgpu::Device, size: (u32, u32)) {
        let want =
            (size.0.div_ceil(ALLOC_STEP) * ALLOC_STEP, size.1.div_ceil(ALLOC_STEP) * ALLOC_STEP);
        let want = (want.0.clamp(ALLOC_STEP, self.max_dim), want.1.clamp(ALLOC_STEP, self.max_dim));
        if let Some(t) = &self.targets {
            // Keep the allocation while it fits and is not wildly oversized —
            // hysteresis, so a drag that oscillates across a step boundary does
            // not thrash.
            let fits = t.alloc.0 >= want.0 && t.alloc.1 >= want.1;
            let snug = t.alloc.0 <= want.0 + 2 * ALLOC_STEP && t.alloc.1 <= want.1 + 2 * ALLOC_STEP;
            if fits && snug {
                return;
            }
        }

        let ext = wgpu::Extent3d { width: want.0, height: want.1, depth_or_array_layers: 1 };
        let make = |label: &str, format, samples, usage| {
            device.create_texture(&wgpu::TextureDescriptor {
                label: Some(label),
                size: ext,
                mip_level_count: 1,
                sample_count: samples,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage,
                view_formats: &[],
            })
        };
        // The resolve target (and blit source) is always single-sampled.
        let color_tex = make(
            "solar-color",
            self.format(),
            1,
            wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        );
        let color = color_tex.create_view(&Default::default());
        let (msaa_tex, msaa) = if self.sample_count > 1 {
            let t = make(
                "solar-msaa",
                self.format(),
                self.sample_count,
                wgpu::TextureUsages::RENDER_ATTACHMENT,
            );
            let v = t.create_view(&Default::default());
            (Some(t), Some(v))
        } else {
            (None, None)
        };
        let depth_tex = make(
            "solar-depth",
            DEPTH_FORMAT,
            self.sample_count,
            wgpu::TextureUsages::RENDER_ATTACHMENT,
        );
        let depth = depth_tex.create_view(&Default::default());

        let blit_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("solar-blit-bg"),
            layout: &self.blit_bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&color),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&self.sampler),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: self.blit_uniform.as_entire_binding(),
                },
            ],
        });

        self.targets = Some(Targets {
            alloc: want,
            color,
            msaa,
            depth,
            blit_bg,
            _color_tex: color_tex,
            _msaa_tex: msaa_tex,
            _depth_tex: depth_tex,
        });
    }

    /// The offscreen colour format, which must match the pipelines' target and
    /// egui's surface so the blit is a straight copy.
    fn format(&self) -> wgpu::TextureFormat {
        self.blit_format
    }

    /// Replace the solar disk image. Rebuilds the scene bind group, which is
    /// why it is generation-counted rather than done per frame.
    pub fn set_sun_image(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        w: u32,
        h: u32,
        rgba: &[u8],
        generation: u64,
    ) {
        if self.sun_gen == generation {
            return;
        }
        if self.sun_tex.width() != w || self.sun_tex.height() != h {
            self.sun_tex = device.create_texture(&wgpu::TextureDescriptor {
                label: Some("solar-sun-tex"),
                size: wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Rgba8UnormSrgb,
                usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
                view_formats: &[],
            });
            self.sun_view = self.sun_tex.create_view(&Default::default());
            self.scene_bg = make_scene_bg(
                device,
                &self.scene_bgl,
                &self.globals_buf,
                &self.land_view,
                &self.sun_view,
                &self.aurora_view,
                &self.border_view,
                &self.river_view,
                &self.city_view,
                &self.body_map_view,
                &self.cloud_view,
                &self.prop_view,
                &self.flash_buf,
                &self.sampler,
            );
        }
        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &self.sun_tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            rgba,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(w * 4),
                rows_per_image: Some(h),
            },
            wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
        );
        self.sun_gen = generation;
    }

    /// Replace the OVATION grid. Fixed size, so unlike the solar image this
    /// never touches the bind group — the generation guard is only there to
    /// keep a 65 kB upload off every frame.
    ///
    /// The grid holds percentages; an `R8Unorm` texel is sampled as `v / 255`.
    /// Rescaling here rather than in the shader is what lets the thresholds in
    /// the WGSL read as the probabilities they are.
    pub fn set_aurora(&mut self, queue: &wgpu::Queue, grid: &[u8], generation: u64) {
        let (w, h) = (sdroxide_solar::aurora::GRID_W, sdroxide_solar::aurora::GRID_H);
        if self.aurora_gen == generation || grid.len() != w * h {
            return;
        }
        // `write_texture` wants rows aligned to 256 bytes; 360 is not, so pad.
        let stride = (w as u32).div_ceil(256) * 256;
        let mut data = vec![0u8; stride as usize * h];
        for row in 0..h {
            let dst = row * stride as usize;
            for col in 0..w {
                let pct = grid[row * w + col].min(100) as u16;
                data[dst + col] = (pct * 255 / 100) as u8;
            }
        }
        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &self.aurora_tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            &data,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(stride),
                rows_per_image: Some(h as u32),
            },
            wgpu::Extent3d { width: w as u32, height: h as u32, depth_or_array_layers: 1 },
        );
        self.aurora_gen = generation;
    }

    /// Replace the propagation heat.
    ///
    /// `rgba` is straight (non-premultiplied) RGBA8, row-major from the north
    /// pole — exactly what `prop_map::PropHeat::rgba` produced for the flat
    /// map. Nothing is recoloured here; see `solar_prop.wgsl` for why.
    ///
    /// Fixed size, so this never touches the bind group. 144 × 4 = 576 bytes a
    /// row, which is not the 256-byte multiple `write_texture` insists on, so
    /// the rows are padded — the same dance the aurora grid does.
    pub fn set_prop(&mut self, queue: &wgpu::Queue, rgba: &[u8], generation: u64) {
        let (w, h) = (sdroxide_types::PROP_GRID_W, sdroxide_types::PROP_GRID_H);
        if self.prop_gen == generation || rgba.len() != w * h * 4 {
            return;
        }
        let stride = ((w * 4) as u32).div_ceil(256) * 256;
        let mut data = vec![0u8; stride as usize * h];
        for row in 0..h {
            let dst = row * stride as usize;
            data[dst..dst + w * 4].copy_from_slice(&rgba[row * w * 4..(row + 1) * w * 4]);
        }
        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &self.prop_tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            &data,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(stride),
                rows_per_image: Some(h as u32),
            },
            wgpu::Extent3d { width: w as u32, height: h as u32, depth_or_array_layers: 1 },
        );
        self.prop_gen = generation;
    }

    /// Upload the cloud field. Fixed size, so this never touches the bind
    /// group; the generation guard keeps a megabyte off every frame.
    ///
    /// Two channels interleaved: red is optical thickness, green is cloud-top
    /// height over 0..[`sdroxide_solar::clouds::TOP_MAX_KM`]. Both are already
    /// bytes in the parsed field, so this is a transpose and nothing else.
    pub fn set_clouds(
        &mut self,
        queue: &wgpu::Queue,
        field: &sdroxide_solar::CloudField,
        generation: u64,
    ) {
        let (w, h) = (sdroxide_solar::clouds::GRID_W, sdroxide_solar::clouds::GRID_H);
        if self.cloud_gen == generation || field.opacity.len() != w * h {
            return;
        }
        let mut data = vec![0u8; w * h * 2];
        for (i, px) in data.chunks_exact_mut(2).enumerate() {
            px[0] = field.opacity[i];
            px[1] = field.top[i];
        }
        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &self.cloud_tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            &data,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(w as u32 * 2),
                rows_per_image: Some(h as u32),
            },
            wgpu::Extent3d { width: w as u32, height: h as u32, depth_or_array_layers: 1 },
        );
        self.cloud_gen = generation;
    }
}

/// Draws one frame of the solar system. Carries the scene by `Arc` so the
/// per-frame handoff is a refcount bump, not a copy of every instance.
pub struct SolarCallback {
    pub scene: std::sync::Arc<Scene>,
    /// Widget size in physical pixels. `prepare` is only given the whole
    /// window's `ScreenDescriptor`, so this has to be measured UI-side.
    pub px_size: [u32; 2],
    /// Latest solar disk image, by `Arc` — several megabytes that must not be
    /// copied per frame. Uploaded only when `sun_gen` changes.
    pub sun_img: Option<std::sync::Arc<sdroxide_solar::SunImage>>,
    pub sun_gen: u64,
    /// Latest auroral oval, on the same terms.
    pub aurora: Option<std::sync::Arc<sdroxide_solar::AuroraOval>>,
    pub aurora_gen: u64,
    /// Latest cloud field, on the same terms.
    pub clouds: Option<std::sync::Arc<sdroxide_solar::CloudField>>,
    pub clouds_gen: u64,
    /// The propagation heat, already resolved to RGBA. `Arc` for the same
    /// reason as the rest: forty kilobytes must not be copied per frame.
    pub prop: Option<std::sync::Arc<Vec<u8>>>,
    pub prop_gen: u64,
}

impl CallbackTrait for SolarCallback {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _screen: &ScreenDescriptor,
        encoder: &mut wgpu::CommandEncoder,
        resources: &mut CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        let Some(r) = resources.get_mut::<SolarResources>() else {
            return Vec::new();
        };
        let size = (self.px_size[0].max(1), self.px_size[1].max(1));

        // Everything that resizes or reallocates happens first, while `r` can
        // still be borrowed mutably; the render pass below borrows the targets
        // immutably and must be last.
        r.ensure_targets(device, size);
        let Some(alloc) = r.targets.as_ref().map(|t| t.alloc) else { return Vec::new() };
        if let Some(img) = &self.sun_img {
            r.set_sun_image(device, queue, img.w, img.h, &img.rgba, self.sun_gen);
        }
        if let Some(oval) = &self.aurora {
            r.set_aurora(queue, &oval.grid, self.aurora_gen);
        }
        if let Some(field) = &self.clouds {
            r.set_clouds(queue, field, self.clouds_gen);
        }
        if let Some(rgba) = &self.prop {
            r.set_prop(queue, rgba, self.prop_gen);
        }

        queue.write_buffer(&r.globals_buf, 0, bytemuck::bytes_of(&self.scene.globals));
        queue.write_buffer(&r.flash_buf, 0, bytemuck::bytes_of(&self.scene.flashes));
        queue.write_buffer(
            &r.blit_uniform,
            0,
            bytemuck::bytes_of(&BlitUniform {
                uv_scale: [
                    size.0 as f32 / alloc.0 as f32,
                    size.1 as f32 / alloc.1 as f32,
                    0.0,
                    0.0,
                ],
            }),
        );

        // Per-draw uniforms, one dynamic-offset slot each.
        let draws = &self.scene.draws;
        if draws.len() as u32 > r.draw_cap {
            r.draw_cap = (draws.len() as u32).next_power_of_two();
            r.draw_buf = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("solar-draws"),
                size: (r.draw_stride * r.draw_cap) as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            r.draw_bg = make_draw_bg(device, &r.draw_bgl, &r.draw_buf);
        }
        for (i, (_, data)) in draws.iter().enumerate() {
            queue.write_buffer(
                &r.draw_buf,
                (i as u32 * r.draw_stride) as u64,
                bytemuck::bytes_of(data),
            );
        }

        write_instances::<LineInst>(
            device,
            queue,
            &mut r.line_buf,
            &mut r.line_cap,
            "solar-lines",
            &self.scene.lines,
        );
        write_instances::<SpriteInst>(
            device,
            queue,
            &mut r.sprite_buf,
            &mut r.sprite_cap,
            "solar-sprites",
            &self.scene.sprites,
        );

        // Everything 3D happens here, in our own pass with our own depth and
        // MSAA — egui's pass never sees any of it.
        let Some(targets) = &r.targets else { return Vec::new() };
        let (view, resolve) = match &targets.msaa {
            Some(msaa) => (msaa, Some(&targets.color)),
            None => (&targets.color, None),
        };
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("solar-scene-pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view,
                depth_slice: None,
                resolve_target: resolve,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color { r: 0.004, g: 0.008, b: 0.018, a: 1.0 }),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &targets.depth,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(DEPTH_CLEAR),
                    store: wgpu::StoreOp::Discard,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        // The pass covers the whole allocation, but only the requested part is
        // ever blitted; restricting the viewport keeps the aspect ratio honest.
        pass.set_viewport(0.0, 0.0, size.0 as f32, size.1 as f32, 0.0, 1.0);
        pass.set_bind_group(0, &r.scene_bg, &[]);

        // Stars first: they are behind everything, and the opaque bodies that
        // follow simply overwrite them.
        if self.scene.draw_stars {
            pass.set_pipeline(&r.sprite_pipe);
            pass.set_vertex_buffer(0, r.quad_vb.slice(..));
            pass.set_vertex_buffer(1, r.star_buf.slice(..));
            pass.draw(0..6, 0..r.star_count);
        }

        // Bodies, then cones — both use per-draw uniforms.
        let mut bound: Option<Prim> = None;
        for (i, (prim, _)) in draws.iter().enumerate() {
            if bound != Some(*prim) {
                match prim {
                    Prim::Sphere => {
                        pass.set_pipeline(&r.body_pipe);
                        pass.set_vertex_buffer(0, r.sphere_vb.slice(..));
                        pass.set_index_buffer(r.sphere_ib.slice(..), wgpu::IndexFormat::Uint32);
                    }
                    // Same mesh as the bodies, different pipeline: the shells
                    // are spheres that add light instead of occluding it.
                    Prim::Aurora => {
                        pass.set_pipeline(&r.aurora_pipe);
                        pass.set_vertex_buffer(0, r.sphere_vb.slice(..));
                        pass.set_index_buffer(r.sphere_ib.slice(..), wgpu::IndexFormat::Uint32);
                    }
                    // Sphere mesh again for both cloud paths: the stack slices
                    // the deck into shells of it, the volume path uses one at
                    // the top of the slab and marches down from there.
                    Prim::Cloud => {
                        pass.set_pipeline(&r.cloud_pipe);
                        pass.set_vertex_buffer(0, r.sphere_vb.slice(..));
                        pass.set_index_buffer(r.sphere_ib.slice(..), wgpu::IndexFormat::Uint32);
                    }
                    Prim::Prop => {
                        pass.set_pipeline(&r.prop_pipe);
                        pass.set_vertex_buffer(0, r.sphere_vb.slice(..));
                        pass.set_index_buffer(r.sphere_ib.slice(..), wgpu::IndexFormat::Uint32);
                    }
                    Prim::CloudVolume => {
                        pass.set_pipeline(&r.cloud_march_pipe);
                        pass.set_vertex_buffer(0, r.sphere_vb.slice(..));
                        pass.set_index_buffer(r.sphere_ib.slice(..), wgpu::IndexFormat::Uint32);
                    }
                    Prim::Cone => {
                        pass.set_pipeline(&r.cone_pipe);
                        pass.set_vertex_buffer(0, r.cone_vb.slice(..));
                        pass.set_index_buffer(r.cone_ib.slice(..), wgpu::IndexFormat::Uint32);
                    }
                    Prim::Ring => {
                        pass.set_pipeline(&r.ring_pipe);
                        pass.set_vertex_buffer(0, r.ring_vb.slice(..));
                        pass.set_index_buffer(r.ring_ib.slice(..), wgpu::IndexFormat::Uint32);
                    }
                    Prim::Tail => {
                        pass.set_pipeline(&r.tail_pipe);
                        pass.set_vertex_buffer(0, r.plume_vb.slice(..));
                        pass.set_index_buffer(r.plume_ib.slice(..), wgpu::IndexFormat::Uint32);
                    }
                }
                bound = Some(*prim);
            }
            pass.set_bind_group(1, &r.draw_bg, &[i as u32 * r.draw_stride]);
            let n = match prim {
                Prim::Sphere | Prim::Aurora | Prim::Cloud | Prim::CloudVolume | Prim::Prop => {
                    r.sphere_indices
                }
                Prim::Cone => r.cone_indices,
                Prim::Ring => r.ring_indices,
                Prim::Tail => r.plume_indices,
            };
            pass.draw_indexed(0..n, 0, 0..1);
        }

        if !self.scene.lines.is_empty() {
            pass.set_pipeline(&r.line_pipe);
            pass.set_vertex_buffer(0, r.quad_vb.slice(..));
            pass.set_vertex_buffer(1, r.line_buf.slice(..));
            pass.draw(0..6, 0..self.scene.lines.len() as u32);
        }
        if !self.scene.sprites.is_empty() {
            pass.set_pipeline(&r.sprite_pipe);
            pass.set_vertex_buffer(0, r.quad_vb.slice(..));
            pass.set_vertex_buffer(1, r.sprite_buf.slice(..));
            pass.draw(0..6, 0..self.scene.sprites.len() as u32);
        }

        Vec::new()
    }

    fn paint(
        &self,
        _info: eframe::egui::PaintCallbackInfo,
        pass: &mut wgpu::RenderPass<'static>,
        resources: &CallbackResources,
    ) {
        let Some(r) = resources.get::<SolarResources>() else { return };
        let Some(t) = &r.targets else { return };
        pass.set_pipeline(&r.blit_pipe);
        pass.set_bind_group(0, &t.blit_bg, &[]);
        pass.draw(0..3, 0..1);
    }
}

/// Grow-and-write an instance buffer, doubling capacity so a scene whose size
/// changes frame to frame does not reallocate constantly.
fn write_instances<T: bytemuck::Pod>(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    buf: &mut wgpu::Buffer,
    cap: &mut usize,
    label: &str,
    items: &[T],
) {
    if items.is_empty() {
        return;
    }
    if items.len() > *cap {
        *cap = items.len().next_power_of_two();
        *buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: (*cap * std::mem::size_of::<T>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
    }
    queue.write_buffer(buf, 0, bytemuck::cast_slice(items));
}

#[cfg(test)]
mod tests {
    // Shader compilation and validation live in `tests/shaders.rs`, which
    // covers the waterfall shaders too and additionally checks the WebGL2
    // uniform-alignment rule.

    /// The vertex-buffer layouts in `build` hard-code these strides and
    /// offsets; if a struct grows, the attributes silently read the wrong
    /// fields rather than failing to compile.
    #[test]
    fn instance_layouts_match_the_structs() {
        use super::super::scene::{LineInst, SpriteInst};
        assert_eq!(std::mem::size_of::<LineInst>(), 48);
        assert_eq!(std::mem::size_of::<SpriteInst>(), 48);
        assert_eq!(std::mem::size_of::<super::mesh::Vertex>(), 20);
    }
}
