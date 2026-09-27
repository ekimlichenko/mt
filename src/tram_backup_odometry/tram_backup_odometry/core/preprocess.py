"""Header-stamp sanitation for the vehicle input streams ('front', 'rear', 'cmd').

The estimator runs on the vehicle header clock (never on receive time), so a bad
header stamp directly moves a sample in time.  ``TimeGuard`` checks every stamp
against the other streams and returns an effective time ``t_eff`` plus a flag.
It does not use any wall or receive clock; the only evidence is the consensus of
the streams themselves and their arrival order.

Notation (per call ``correct(s, t)``; ``n`` counts all messages):

    last[s]   raw header of the last message of stream s (updated on every message)
    seen[s]   value of n at that message
    base[s]   last header of s that was *not* glitch-corrected (the stream's own clock)
    now       latest accepted effective time (the filter clock), -inf at start
    R         = max(last[s'] for s' != s with n - seen[s'] <= time_consensus_n)
                reference clock of the other live streams (None if there is none)

    forward glitch  <=>  R exists  and  t > R + g  and  t - base[s] > g + 0.15  and  t > now
                         (g = time_glitch_s; base[s] unknown counts as a jump; the last
                         condition keeps t_eff < t_hdr as the contract requires)
                         -> t_eff = max(R, now), flag 1.  The arrival order is trusted,
                         the stamp is not.  After more than time_resync_n consecutive
                         glitches of the same stream its new clock is accepted.
    late            <=>  t < now - time_late_tol_s  -> (t, 2): older than what the filter
                         already processed; the pipeline must not feed it to the filter.
    otherwise       ->   (t, 0), now = max(now, t)
    invalid stamp   ->   non-finite or <= 0: (t, 3), the message is dropped.

``last`` stores the raw header in every case, so two streams that jump together
(the front and rear bogie share one source frame and identical stamps) confirm
each other and re-establish consensus on the new clock.  ``base`` deliberately
skips corrected stamps, so a persistent offset of a single stream keeps being
detected and the resync counter can reach its limit.

Tolerance of the late test: the wheel stamps trail the 20 Hz command stamps by
the wheel latency plus jitter.  Over the 92 recorded bags without clock glitches
(97 unique bags minus 5), after the first second, the running header maximum
leads the stamp of an arriving message by up to 0.101 s (p99 0.086 s; 10.07 % of
the messages exceed 0.05 s, none 0.15 s; tools/calibration/timeguard_flags.py),
so the default tolerance is 0.15 s; only genuine clock glitches exceed it.
"""
import math

STREAMS = ('front', 'rear', 'cmd')

FLAG_OK = 0
FLAG_FORWARD = 1
FLAG_LATE = 2
FLAG_INVALID = 3


class TimeGuard:
    """Header-time consensus across input streams ('front', 'rear', 'cmd'); clock-agnostic (no receive time)."""

    def __init__(self, params):
        self.glitch_s = float(getattr(params, 'time_glitch_s', 0.3))
        self.jump_margin_s = float(getattr(params, 'time_jump_margin_s', 0.15))
        self.consensus_n = int(getattr(params, 'time_consensus_n', 30))
        self.resync_n = int(getattr(params, 'time_resync_n', 20))
        self.late_tol_s = float(getattr(params, 'time_late_tol_s', 0.15))
        self._last = {}
        self._seen = {}
        self._base = {}
        self._run = {}
        self._n = 0
        self._now = -math.inf
        self.counts = [0, 0, 0, 0]
        self.resyncs = 0

    @property
    def now(self) -> float:
        return self._now

    def correct(self, stream: str, t_hdr: float) -> tuple:
        """Return (t_eff, flag) for a message of ``stream`` stamped ``t_hdr``."""
        t = float(t_hdr)
        if not math.isfinite(t) or t <= 0.0:
            self.counts[FLAG_INVALID] += 1
            return t, FLAG_INVALID
        self._n += 1
        n = self._n

        ref = None
        for s, t_s in self._last.items():
            if s != stream and n - self._seen[s] <= self.consensus_n and (ref is None or t_s > ref):
                ref = t_s
        base = self._base.get(stream)
        self._last[stream] = t
        self._seen[stream] = n

        # t > now keeps flag 1 a pure backward shift (t_eff < t_hdr): a stamp that is ahead of a
        # stale reference but not of the filter clock is handled by the late/ok tests below
        if (ref is not None and t > ref + self.glitch_s and t > self._now
                and (base is None or t - base > self.glitch_s + self.jump_margin_s)):
            run = self._run.get(stream, 0) + 1
            if run <= self.resync_n:
                self._run[stream] = run
                t_eff = ref if ref > self._now else self._now
                self._now = t_eff
                self.counts[FLAG_FORWARD] += 1
                return t_eff, FLAG_FORWARD
            self.resyncs += 1
        self._run[stream] = 0
        self._base[stream] = t

        if t < self._now - self.late_tol_s:
            self.counts[FLAG_LATE] += 1
            return t, FLAG_LATE
        if t > self._now:
            self._now = t
        self.counts[FLAG_OK] += 1
        return t, FLAG_OK
