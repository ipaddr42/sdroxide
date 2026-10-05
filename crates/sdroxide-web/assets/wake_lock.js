// Screen wake lock for the sdroxide web client.
//
// A receiver panel is watched, not read: the operator looks at a waterfall and
// an S-meter for minutes at a time without touching the glass, and a phone or
// tablet blanks after fifteen or thirty seconds of that. Installed full screen
// the page owns the whole panel, so the blank screen is the whole application
// going away, and it comes back behind a lock screen.
//
// The browser releases the lock itself whenever the page is hidden — which is
// the behaviour wanted, since a backgrounded client should not hold the screen
// on — so the only work here is taking it again on the way back.
//
// Secure-context only, like the audio path, and a sentinel cannot be requested
// until the document is visible. Unsupported browsers simply never acquire one;
// this is a comfort, never a requirement, so nothing here reports an error to
// the operator.

(function () {
    let sentinel = null;
    // The request in flight, if any. A request is a promise, and the guard
    // below is on the far side of an await: without this a load and a first
    // touch a few milliseconds apart both pass it, the browser hands out two
    // sentinels, and only the second is ever tracked — so `set(false)` would
    // release one lock and leave the screen held by the other.
    let pending = null;
    let wanted = true;

    const SUPPORTED = window.isSecureContext && "wakeLock" in navigator;

    async function acquire() {
        if (!SUPPORTED || !wanted || sentinel || pending || document.hidden) return;
        try {
            pending = navigator.wakeLock.request("screen");
            const s = await pending;
            // Dropped by the browser on hide, on a tab switch, or when the
            // system decides otherwise. Clearing it here is what lets the
            // visibility handler below take a fresh one — and only when it is
            // still the one being held, so a stale release cannot clear a
            // newer lock and leave it untracked.
            s.addEventListener("release", function () {
                if (sentinel === s) sentinel = null;
            });
            sentinel = s;
        } catch (e) {
            // Denied (battery saver, a policy, no user activation yet). Not an
            // error worth showing: the screen simply sleeps as it would have.
            sentinel = null;
        } finally {
            pending = null;
        }
    }

    async function release() {
        // A request still in flight resolves into `sentinel` after this runs,
        // so wait for it rather than walking away from a lock about to be
        // handed to us.
        if (pending) {
            try {
                await pending;
            } catch (e) {
                /* never arrived */
            }
        }
        if (!sentinel) return;
        const s = sentinel;
        sentinel = null;
        try {
            await s.release();
        } catch (e) {
            /* already gone */
        }
    }

    // Some browsers refuse a lock that no gesture asked for, so try on load and
    // again on the first interaction; whichever succeeds first wins, and the
    // second call is a no-op while a sentinel is held.
    document.addEventListener("visibilitychange", function () {
        if (document.hidden) return;
        acquire();
    });
    for (const ev of ["pointerdown", "keydown", "touchend"]) {
        window.addEventListener(ev, acquire, { passive: true });
    }
    acquire();

    // For the client to drive if it ever grows a setting for this: the panel is
    // the only thing that knows whether it is receiving or idle.
    window.sdroxideWakeLock = {
        set: function (on) {
            wanted = !!on;
            if (wanted) acquire();
            else release();
        },
        active: function () {
            return sentinel !== null;
        },
        supported: function () {
            return SUPPORTED;
        },
    };
})();
