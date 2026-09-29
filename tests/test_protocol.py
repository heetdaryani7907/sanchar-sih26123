"""Wire protocol: signing, tamper detection, anti-replay window, message size."""
from sanchar.protocol.messages import (Envelope, Hello, Intent, ReplayGuard, Signer, TaskAnnounce,
                                       decode, encode)


def _hello(**kw):
    base = dict(robot_id=3, tick=10, node=42, priority=(0, 5, -3, -3, 0), battery=0.8, claim=43,
                heard={i: 9 for i in range(12)})
    base.update(kw)
    return Hello(**base)


def test_sign_verify_roundtrip():
    s = Signer(b"fleet-key")
    env = s.sign(Envelope(ver=1, sender=3, seq=1, hlc=0, hops=0, body=_hello()))
    got = decode(encode(env))
    assert s.verify(got)
    assert isinstance(got.body, Hello) and got.body.claim == 43


def test_tampered_or_foreign_key_rejected():
    s = Signer(b"fleet-key")
    env = s.sign(Envelope(ver=1, sender=3, seq=1, hlc=0, hops=0, body=_hello()))
    forged = decode(encode(env))
    forged.body = _hello(claim=99)                     # attacker rewrites the claim
    assert not s.verify(forged)
    assert not Signer(b"other-key").verify(decode(encode(env)))


def test_replay_window_accepts_each_seq_once_and_tolerates_reordering():
    g = ReplayGuard(window=8)
    assert [g.accept(1, x) for x in (5, 7, 6, 6, 3)] == [True, True, True, False, True]
    assert g.accept(1, 20) and not g.accept(1, 11)      # too old after the window moved
    assert g.accept(2, 1)                               # senders are independent


def test_messages_are_small():
    s = Signer()
    hello = encode(s.sign(Envelope(ver=1, sender=3, seq=1, hlc=0, hops=0, body=_hello())))
    intent = encode(s.sign(Envelope(ver=1, sender=3, seq=2, hlc=0, hops=0,
                                    body=Intent(robot_id=3, tick=10, nodes=list(range(11)), priority=(0, 1, 2, 3, 0)))))
    task = encode(s.sign(Envelope(ver=1, sender=3, seq=3, hlc=0, hops=0,
                                  body=TaskAnnounce(task_id="T0001", pickup=5, dropoff=9, created_tick=1, origin=-1))))
    assert len(hello) < 220, len(hello)                # 12-peer ack map included
    assert len(intent) < 140 and len(task) < 200, (len(intent), len(task))
