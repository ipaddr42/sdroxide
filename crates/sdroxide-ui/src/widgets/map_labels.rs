//! Names beside the symbols on a map — the AIS chart's vessel names, the ADS-B
//! picture's data blocks — placed so that they never pile up.
//!
//! Both maps used to have an all-or-nothing rule: every name while few enough
//! targets were in view, none at all above a fixed count. The count in view
//! changes as the map pans and zooms, so names came and went with where the map
//! was rather than with the traffic (issue #408). Here each name is placed
//! greedily instead, the way [`super::worldmap`] places city names: the ones
//! that must show first, then the rest by rank, each only if it misses every
//! name already placed.
//!
//! A name goes up and to the right of its symbol, where every radar and chart
//! plotter puts it, and moves to another corner only when that one has no room
//! — at the map's edge above all. The auto-fit leaves the outermost target a
//! tenth of the map from the edge, which is less than a long name needs, so a
//! name that could only sit up and to the right went missing on exactly the
//! targets that frame the picture.
//!
//! Every symbol is in the way too, not only the names: a name drawn over a
//! neighbour hides the one thing on the map that says where that neighbour is.

use std::ops::RangeInclusive;

use eframe::egui::{Color32, FontId, Painter, Pos2, Rect, Vec2, pos2, vec2};

/// From one line of a name to the next, in points.
const PITCH: f32 = 10.0;

/// How far a symbol's halo reaches past its radius. Both maps draw a disc of
/// sea this far out round every symbol, so a name over the halo hides the
/// symbol as surely as one over the symbol itself.
const HALO: f32 = 2.0;

/// Side of one cell of [`Occupancy`], in points: about one short name.
const CELL: f32 = 48.0;

/// One target's name, waiting to be placed.
pub struct MapLabel {
    /// The symbol's centre.
    pub at: Pos2,
    /// The symbol's radius. The name starts just beyond it.
    pub r: f32,
    /// Where the tick joining name and symbol starts, as a fraction of `r`
    /// along the diagonal toward the name.
    pub tick_from: f32,
    /// The tick's colour.
    pub tick: Color32,
    /// The lines, top to bottom, each in its own colour.
    pub lines: Vec<(String, Color32)>,
    /// Drawn whatever it lands on: the target the operator picked or is
    /// pointing at, or one in distress. Placed first, so a crowded map culls
    /// *around* it rather than through it.
    pub must: bool,
    /// How hard the name fights for room: lower wins.
    pub rank: u8,
    /// Settles a tie in rank — the MMSI, the ICAO address. The trackers hand
    /// targets over in hash map order, so breaking ties by position would let
    /// which of two neighbours keeps its name change whenever the map is
    /// rehashed.
    pub key: u32,
}

/// Which corner of its symbol a name sits off.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Corner {
    UpRight,
    UpLeft,
    DownRight,
    DownLeft,
}

impl Corner {
    /// In the order tried: the usual place first, then the mirror image across
    /// the symbol, so a name crowded off the map's right edge goes left before
    /// it goes below.
    const ALL: [Corner; 4] = [Corner::UpRight, Corner::UpLeft, Corner::DownRight, Corner::DownLeft];

    /// Which way the name lies from the symbol: +1 right or down, −1 left or up.
    fn signs(self) -> (f32, f32) {
        match self {
            Corner::UpRight => (1.0, -1.0),
            Corner::UpLeft => (-1.0, -1.0),
            Corner::DownRight => (1.0, 1.0),
            Corner::DownLeft => (-1.0, 1.0),
        }
    }
}

/// A name reduced to what placing it needs: how big it is, where its symbol
/// is, and what decides whether it gets room.
struct Slot {
    at: Pos2,
    r: f32,
    /// The whole block, every line of it.
    size: Vec2,
    /// One line's height.
    line_h: f32,
    must: bool,
    rank: u8,
    key: u32,
}

impl Slot {
    /// The symbol with its halo. A name never reaches its own — the block
    /// starts two points past the halo — so every symbol can stand in the way
    /// of every name without asking whose it is.
    fn symbol(&self) -> Rect {
        Rect::from_center_size(self.at, Vec2::splat(2.0 * (self.r + HALO)))
    }

    /// Where the tick joining name and symbol ends.
    fn anchor(&self, corner: Corner) -> Pos2 {
        let (sx, sy) = corner.signs();
        self.at + vec2(sx * (self.r + 3.0), sy * (self.r + 2.0))
    }

    /// The block's box off `corner`. Above the symbol each line is centred
    /// half a line above the tick's end, so the last one sits on the tick
    /// rather than being cut through by it; below, the same mirrored. To the
    /// left the lines are set flush right, against the tick.
    fn block(&self, corner: Corner) -> Rect {
        let (sx, sy) = corner.signs();
        let anchor = self.anchor(corner);
        let left = if sx > 0.0 { anchor.x + 2.0 } else { anchor.x - 2.0 - self.size.x };
        let top = if sy < 0.0 {
            anchor.y - PITCH / 2.0 + self.line_h / 2.0 - self.size.y
        } else {
            anchor.y + PITCH / 2.0 - self.line_h / 2.0
        };
        Rect::from_min_size(pos2(left, top), self.size)
    }

    /// The room the block claims, with a point to spare all round.
    fn area(&self, corner: Corner) -> Rect {
        self.block(corner).expand(1.0)
    }
}

/// Boxes already on the map — the symbols and marks, or the names placed so
/// far — filed by the cells of a coarse grid, so asking whether a name would
/// hit one looks at its few neighbours rather than at everything on a map that
/// may carry hundreds of targets.
struct Occupancy {
    origin: Pos2,
    cols: usize,
    rows: usize,
    cells: Vec<Vec<Rect>>,
}

impl Occupancy {
    fn new(bounds: Rect) -> Self {
        let cols = (bounds.width() / CELL).ceil().max(1.0) as usize;
        let rows = (bounds.height() / CELL).ceil().max(1.0) as usize;
        Self { origin: bounds.min, cols, rows, cells: vec![Vec::new(); cols * rows] }
    }

    /// The columns and rows `r` covers. A box past the edge is filed in the
    /// edge cells, which only costs a comparison or two when asked about.
    fn span(&self, r: Rect) -> (RangeInclusive<usize>, RangeInclusive<usize>) {
        let cell =
            |v: f32, o: f32, n: usize| (((v - o) / CELL).floor().max(0.0) as usize).min(n - 1);
        let (o, c, n) = (self.origin, self.cols, self.rows);
        (
            cell(r.min.x, o.x, c)..=cell(r.max.x, o.x, c),
            cell(r.min.y, o.y, n)..=cell(r.max.y, o.y, n),
        )
    }

    fn insert(&mut self, r: Rect) {
        let (xs, ys) = self.span(r);
        for y in ys {
            for x in xs.clone() {
                self.cells[y * self.cols + x].push(r);
            }
        }
    }

    fn hits(&self, r: Rect) -> bool {
        let (xs, ys) = self.span(r);
        ys.into_iter().any(|y| {
            xs.clone().any(|x| self.cells[y * self.cols + x].iter().any(|q| q.intersects(r)))
        })
    }
}

/// Which of `slots` to draw, by index, and off which corner, in draw order.
///
/// The ones that must appear first — at a corner with room where there is one,
/// so a selected target at the map's edge still shows its whole name. Where
/// there is none, one that at least misses the other names, so a selected
/// vessel beside one in distress in a crowded harbour does not write over it;
/// and where there is not even that, wherever it lands. Then the rest in rank
/// order, each at the first corner whose box stays inside `bounds` and misses
/// every symbol, every one of `marks` and every name already placed, or not at
/// all.
fn plan(slots: &[Slot], marks: &[Rect], bounds: Rect) -> Vec<(usize, Corner)> {
    let mut symbols = Occupancy::new(bounds);
    for r in slots.iter().map(Slot::symbol).chain(marks.iter().copied()) {
        symbols.insert(r);
    }
    let mut names = Occupancy::new(bounds);
    let mut order: Vec<usize> = (0..slots.len()).collect();
    order.sort_by_key(|&i| (!slots[i].must, slots[i].rank, slots[i].key));
    let mut out = Vec::new();
    for i in order {
        let slot = &slots[i];
        let inside = |c: &Corner| bounds.contains_rect(slot.area(*c));
        let unnamed = |c: &Corner| inside(c) && !names.hits(slot.area(*c));
        let clear = |c: &Corner| unnamed(c) && !symbols.hits(slot.area(*c));
        let corner = match Corner::ALL.iter().find(|c| clear(c)) {
            Some(&c) => c,
            None if slot.must => Corner::ALL
                .into_iter()
                .find(unnamed)
                .or_else(|| Corner::ALL.into_iter().find(inside))
                .unwrap_or(Corner::UpRight),
            None => continue,
        };
        names.insert(slot.area(corner));
        out.push((i, corner));
    }
    out
}

/// Lay out every name in `labels`, place the ones that fit inside `bounds`,
/// and draw them — each off a corner of its symbol, with a tick joining the
/// two so a crowded picture still says which name is whose. `marks` are
/// anything else on the map a name must keep off, such as our own position.
pub fn draw(p: &Painter, bounds: Rect, font: &FontId, labels: Vec<MapLabel>, marks: &[Rect]) {
    let mut slots = Vec::with_capacity(labels.len());
    let mut laid = Vec::with_capacity(labels.len());
    for label in labels {
        let galleys: Vec<_> = label
            .lines
            .into_iter()
            .map(|(text, colour)| (p.layout_no_wrap(text, font.clone(), colour), colour))
            .collect();
        if galleys.is_empty() {
            continue;
        }
        let n = galleys.len();
        let line_h = galleys.iter().map(|(g, _)| g.size().y).fold(0.0, f32::max);
        let w = galleys.iter().map(|(g, _)| g.size().x).fold(0.0, f32::max);
        slots.push(Slot {
            at: label.at,
            r: label.r,
            size: vec2(w, (n - 1) as f32 * PITCH + line_h),
            line_h,
            must: label.must,
            rank: label.rank,
            key: label.key,
        });
        laid.push((label.tick_from, label.tick, galleys));
    }
    for (i, corner) in plan(&slots, marks, bounds) {
        let slot = &slots[i];
        let (tick_from, tick_colour, galleys) = &laid[i];
        let (sx, sy) = corner.signs();
        let t = slot.r * tick_from;
        p.line_segment([slot.at + vec2(sx * t, sy * t), slot.anchor(corner)], (1.0, *tick_colour));
        let block = slot.block(corner);
        for (k, (galley, colour)) in galleys.iter().enumerate() {
            let x = if sx > 0.0 { block.left() } else { block.right() - galley.size().x };
            p.galley(pos2(x, block.top() + k as f32 * PITCH), galley.clone(), *colour);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bounds() -> Rect {
        Rect::from_min_size(pos2(0.0, 0.0), vec2(200.0, 100.0))
    }

    /// A two-line name 40 points wide, 10-point lines, off a symbol at `(x, y)`.
    fn slot(x: f32, y: f32, must: bool, rank: u8) -> Slot {
        Slot { at: pos2(x, y), r: 5.0, size: vec2(40.0, 20.0), line_h: 10.0, must, rank, key: 0 }
    }

    fn only(placed: &[(usize, Corner)]) -> Vec<usize> {
        placed.iter().map(|&(i, _)| i).collect()
    }

    /// The usual corner is where the names have always sat: the last line
    /// centred half a line above the tick's end, the block starting two points
    /// right of it.
    #[test]
    fn a_name_sits_up_and_to_the_right_where_it_always_did() {
        let s = slot(100.0, 50.0, false, 1);
        let anchor = s.anchor(Corner::UpRight);
        assert_eq!(anchor, pos2(108.0, 43.0));
        let block = s.block(Corner::UpRight);
        assert_eq!(block.left(), anchor.x + 2.0);
        // Last line's middle = anchor.y - 5; its bottom half a line below that.
        assert_eq!(block.bottom(), anchor.y - 5.0 + 5.0);
        assert_eq!(plan(&[s], &[], bounds()), vec![(0, Corner::UpRight)]);
    }

    /// A name with no room up and to the right — at the map's right or top
    /// edge — moves to a corner that has some rather than vanishing. The
    /// auto-fit puts the outermost target that close to the edge, so this is
    /// the targets that frame the picture, on a map with nothing else on it.
    #[test]
    fn a_name_at_the_edge_moves_to_a_corner_with_room() {
        assert_eq!(plan(&[slot(190.0, 50.0, false, 1)], &[], bounds()), vec![(0, Corner::UpLeft)]);
        assert_eq!(plan(&[slot(20.0, 5.0, false, 1)], &[], bounds()), vec![(0, Corner::DownRight)]);
        assert_eq!(plan(&[slot(190.0, 5.0, false, 1)], &[], bounds()), vec![(0, Corner::DownLeft)]);
        // Flush right when it is on the left, so it still hugs its tick.
        let s = slot(190.0, 50.0, false, 1);
        assert_eq!(s.block(Corner::UpLeft).right(), s.anchor(Corner::UpLeft).x - 2.0);
        // Below, the first line sits as far under the tick as the last one sits
        // over it above — whatever the font makes a line's height.
        let s = Slot { line_h: 12.0, size: vec2(40.0, 22.0), ..slot(100.0, 50.0, false, 1) };
        let below = s.block(Corner::DownRight).top() - s.anchor(Corner::DownRight).y;
        let above = s.anchor(Corner::UpRight).y - s.block(Corner::UpRight).bottom();
        assert_eq!(below, above);
    }

    /// A name keeps off other symbols and off our own mark, not only off other
    /// names: over a neighbour it would hide where that neighbour is.
    #[test]
    fn a_name_keeps_off_other_symbols_and_marks() {
        // The second symbol sits where the first one's name would go.
        let slots = vec![slot(100.0, 50.0, false, 1), slot(130.0, 35.0, false, 3)];
        assert_eq!(plan(&slots, &[], bounds()), vec![(0, Corner::UpLeft), (1, Corner::UpRight)]);
        // Our own position, likewise.
        let home = Rect::from_center_size(pos2(130.0, 35.0), vec2(15.0, 15.0));
        let slots = vec![slot(100.0, 50.0, false, 1)];
        assert_eq!(plan(&slots, &[home], bounds()), vec![(0, Corner::UpLeft)]);
    }

    /// A busy map, placed through the grid, gives exactly what checking every
    /// box against every other would: each name at the first corner with room
    /// when its turn comes, and none where no corner had any.
    #[test]
    fn the_grid_places_a_busy_map_as_checking_everything_would() {
        let bounds = Rect::from_min_size(pos2(12.0, 7.0), vec2(700.0, 380.0));
        let mut seed = 0x2545_f491_u32;
        let mut next = move |n: u32| {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (seed >> 8) % n
        };
        let slots: Vec<Slot> = (0..400)
            .map(|k| {
                let lines = 1 + next(3) as usize;
                Slot {
                    at: pos2(bounds.left() + next(700) as f32, bounds.top() + next(380) as f32),
                    r: 3.5 + next(3) as f32,
                    size: vec2(24.0 + next(90) as f32, (lines - 1) as f32 * PITCH + 11.0),
                    line_h: 11.0,
                    must: next(50) == 0,
                    rank: next(4) as u8,
                    key: k,
                }
            })
            .collect();
        let marks = [Rect::from_center_size(bounds.center(), vec2(15.0, 15.0))];
        let placed = plan(&slots, &marks, bounds);

        let mut order: Vec<usize> = (0..slots.len()).collect();
        order.sort_by_key(|&i| (!slots[i].must, slots[i].rank, slots[i].key));
        let symbols: Vec<Rect> = slots.iter().map(Slot::symbol).collect();
        let mut names: Vec<Rect> = Vec::new();
        let mut next_placed = placed.iter().peekable();
        for i in order {
            let s = &slots[i];
            let clear = |c: Corner| {
                let a = s.area(c);
                bounds.contains_rect(a)
                    && !symbols.iter().chain(&marks).chain(&names).any(|q| q.intersects(a))
            };
            let first_clear = Corner::ALL.into_iter().find(|&c| clear(c));
            match next_placed.next_if(|&&(j, _)| j == i) {
                Some(&(_, corner)) => {
                    if !s.must || first_clear.is_some() {
                        assert_eq!(Some(corner), first_clear, "slot {i}");
                    }
                    names.push(s.area(corner));
                }
                None => assert_eq!(first_clear, None, "slot {i} was dropped with room"),
            }
        }
        assert!(next_placed.next().is_none());
        // Busy enough that most names are culled, not so busy that none fit.
        assert!((20..200).contains(&placed.len()), "{} placed", placed.len());
    }

    /// A map too busy for every name keeps the ones that say the most, by
    /// rank — the opposite of the rule this replaced, which dropped them all at
    /// once once a few dozen targets were in view (issue #408).
    #[test]
    fn a_crowded_map_keeps_the_names_that_matter() {
        // A map with room for one name, beside one spot, and two symbols on
        // it: the better-ranked one wins.
        let narrow = Rect::from_min_size(pos2(0.0, 0.0), vec2(60.0, 32.0));
        let slots = vec![slot(4.0, 30.0, false, 3), slot(4.0, 30.0, false, 1)];
        assert_eq!(plan(&slots, &[], narrow), vec![(1, Corner::UpRight)]);
        // Two symbols on top of each other on an open map: both names fit, one
        // either side.
        let slots = vec![slot(100.0, 50.0, false, 3), slot(100.0, 50.0, false, 1)];
        assert_eq!(plan(&slots, &[], bounds()), vec![(1, Corner::UpRight), (0, Corner::UpLeft)]);
    }

    /// Two names of the same rank after the same room: the lower key wins,
    /// whichever order the tracker happened to list them in.
    #[test]
    fn a_tie_in_rank_goes_to_the_lower_key_not_the_list_order() {
        let narrow = Rect::from_min_size(pos2(0.0, 0.0), vec2(60.0, 32.0));
        let a = || Slot { key: 244_000_002, ..slot(4.0, 30.0, false, 1) };
        let b = || Slot { key: 244_000_001, ..slot(4.0, 30.0, false, 1) };
        assert_eq!(only(&plan(&[a(), b()], &[], narrow)), vec![1]);
        assert_eq!(only(&plan(&[b(), a()], &[], narrow)), vec![0]);
    }

    /// The name the operator picked or is pointing at is placed first and drawn
    /// whatever else is around it, so a crowded map culls *around* it rather
    /// than through it.
    #[test]
    fn the_name_the_operator_is_looking_at_always_shows() {
        // A must-label takes the room from a better-ranked one...
        let narrow = Rect::from_min_size(pos2(0.0, 0.0), vec2(60.0, 32.0));
        let slots = vec![slot(4.0, 30.0, true, 3), slot(4.0, 30.0, false, 0)];
        assert_eq!(only(&plan(&slots, &[], narrow)), vec![0]);
        // ...and where no corner has room it is drawn anyway, up and to the
        // right, where the painter clips it...
        let tiny = Rect::from_min_size(pos2(0.0, 0.0), vec2(20.0, 20.0));
        assert_eq!(plan(&[slot(10.0, 10.0, true, 3)], &[], tiny), vec![(0, Corner::UpRight)]);
        // ...but two of them hemmed in by symbols on every side still keep off
        // each other's names: a selected vessel beside one in distress in a
        // crowded harbour must not write over it.
        let mut slots = vec![slot(100.0, 50.0, true, 0), slot(100.0, 50.0, true, 1)];
        for (x, y) in [(130.0, 35.0), (70.0, 35.0), (130.0, 65.0), (70.0, 65.0)] {
            slots.push(slot(x, y, false, 3));
        }
        let placed = plan(&slots, &[], bounds());
        let area = |i: usize| {
            let &(_, c) = placed.iter().find(|&&(j, _)| j == i).expect("placed");
            slots[i].area(c)
        };
        assert!(!area(0).intersects(area(1)), "{:?} over {:?}", area(1), area(0));
    }

    /// A name with no room anywhere on the map does not show, unless it must.
    #[test]
    fn a_name_with_no_room_is_dropped_unless_it_must_show() {
        let tiny = Rect::from_min_size(pos2(0.0, 0.0), vec2(20.0, 20.0));
        assert!(plan(&[slot(10.0, 10.0, false, 0)], &[], tiny).is_empty());
    }
}
