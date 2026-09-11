//! The station's shared-device registry: one physical device, many radios.
//!
//! A multi-stream device — a TCI rig with two receivers, later an HPSDR
//! board's DDCs or a Pluto's second chain — carries one *connection* that
//! several radio tabs draw streams from. Whichever radio opens first creates
//! the connection; the others attach to it. The registry is how they find it:
//! keyed by the device's identity, holding a [`Weak`] so it never keeps a
//! device alive by itself — the sources' own `Arc`s do that, and dropping the
//! last one closes the connection. Releasing one radio's stream therefore
//! never tears down another's.
//!
//! Lives in the binary, like the backends themselves: the engine stays free
//! of process globals, and each engine's [`sdroxide_radio::ReopenFn`] simply
//! calls in here. The one lock also serializes two engines' reconnect workers
//! racing to revive the same dead rig — the first one reopens, the second
//! finds the fresh entry and attaches.

use std::any::Any;
use std::collections::HashMap;
use std::sync::{Mutex, OnceLock, Weak};

/// A shareable device's identity — what makes two radios' configurations
/// "the same device".
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum DeviceKey {
    /// A TCI server, by the address dialed ("host:port").
    Tci(String),
    /// An OpenHPSDR board, by its IP.
    Hpsdr(std::net::Ipv4Addr),
    /// A PlutoSDR, by the address dialed (`host[:port]`).
    Pluto(String),
    /// An SDRplay RSP, by the serial the API service reports.
    ///
    /// Always the *resolved* serial — `SdrPlaySource::open` turns "the first
    /// one found" into the receiver it actually found before keying on it.
    /// Keyed on the configured string instead, two radios meaning one RSPduo
    /// missed each other whenever one of them named the board and the other
    /// did not, and the second went off to open a session on a device this
    /// process already held (issue #392). Falls back to the configured string
    /// only when nothing is there to resolve against, where the open is going
    /// to fail anyway and should say so itself.
    SdrPlay(String),
}

/// What every entry must answer: whether the connection behind it still
/// works. A dead one is replaced on the next open rather than handed out.
pub trait SharedDevice: Any + Send + Sync {
    fn is_alive(&self) -> bool;
}

#[derive(Default)]
pub struct DeviceRegistry {
    inner: Mutex<HashMap<DeviceKey, Weak<dyn SharedDevice>>>,
}

impl DeviceRegistry {
    /// The live device for `key`, opening one if there is none (or only a
    /// dead or dropped one). The registry lock is held across `open`, which
    /// is the point: two radios reviving the same rig at once must not both
    /// dial it — the loser of the race finds the winner's connection here.
    pub fn get_or_open<T, F>(&self, key: DeviceKey, open: F) -> Result<std::sync::Arc<T>, String>
    where
        T: SharedDevice,
        F: FnOnce() -> Result<std::sync::Arc<T>, String>,
    {
        let mut map = self.inner.lock().expect("device registry lock");
        if let Some(existing) = map.get(&key).and_then(Weak::upgrade)
            && existing.is_alive()
        {
            let any: std::sync::Arc<dyn Any + Send + Sync> = existing;
            match any.downcast::<T>() {
                Ok(dev) => return Ok(dev),
                // A different type under the same key cannot happen — the key
                // names the backend — but a wrong branch here must open a
                // fresh device, not hand back nonsense.
                Err(_) => {}
            }
        }
        let dev = open()?;
        map.insert(key, std::sync::Arc::downgrade(&dev) as Weak<dyn SharedDevice>);
        Ok(dev)
    }
}

/// The process-wide registry. A static rather than a handle threaded through
/// every open path: sources are built from `main`, from each engine's reopen
/// factory and from each reconnect worker, and all of them mean the same
/// station.
pub fn registry() -> &'static DeviceRegistry {
    static REGISTRY: OnceLock<DeviceRegistry> = OnceLock::new();
    REGISTRY.get_or_init(DeviceRegistry::default)
}
