# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The last thing between a typed frame and a motor.

WHY THIS IS NOT IN THE PARSER
-----------------------------
Until now every safety check lived inside `interpret` -- the question guard,
the prohibition guard, the self-negation check, the plausibility rail. That
works exactly as long as the parser is the only thing producing frames.

The target architecture does not keep that property. Once a model
interprets the messy 70% the parser cannot -- typos, retraction,
paraphrase, hypotheticals -- those guards are no longer on the path, and a
model makes the same CLASS of mistake the keyword ladder made. Measured on
this project's own history: the ladder emitted `stop()` for "how many times
have you had to stop?". A model will do that less often and less
predictably, and unlike the ladder you cannot read why.

So the checks move HERE, where they run on `(utterance, frames)` and do not
care who produced the frames. That is strictly stronger than where they
were: it protects against the parser, against a model, and against whatever
interprets next.

    from omnisim_bridges.gate import check
    rejected = check("how many times did you stop?", frames)

WHAT IT ENFORCES
    1. an interrogative may not produce motion
    2. a prohibition may not produce the thing it prohibits
    3. a self-negating order ("drive 1 m without moving") produces nothing
    4. magnitudes stay inside a sanity rail
    5. a direction word and a sign must agree
    6. args match the tool's declared schema -- required present, types
       right, numbers finite
    7. an unresolved referent ("it", "there") never reaches an actuator
    8. a confirm-required tool needs an authorization token

⚠️ This is a SANITY gate, not a world model. It does not know the arena, so
rule 4 is a rail rather than a bound; a world-aware clamp still belongs in
the bridge, where `act_drive_forward` and `act_drive_to` have no limit of
any kind.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

__all__ = ["check", "Rejection", "SPECS", "register_tools",
           "reject_toolcall", "is_physical",
           "MAX_DISTANCE_M", "MAX_ANGLE_RAD"]

MAX_DISTANCE_M = 50.0
MAX_ANGLE_RAD = 8 * math.pi          # four full turns
# A Husky tops out near 1 m/s and the fastest platform here is well under
# 5. The rail is not a performance limit, it is a "nobody meant this" limit.
MAX_SPEED_MPS = 5.0
MAX_YAW_RATE_RPS = 6.0        # ~1 revolution/second
MAX_ALTITUDE_M = 120.0        # the usual legal ceiling for small UAS
MAX_BODY_SHIFT_M = 1.0        # a quadruped body shift, not a walk

SAFE, GUARDED, CONFIRM = "safe", "guarded", "confirm_required"


@dataclass(frozen=True)
class Rejection:
    """One reason a frame may not be dispatched."""
    tool: str
    rule: str
    detail: str

    def __str__(self) -> str:          # pragma: no cover - display only
        return f"{self.tool}: {self.detail} [{self.rule}]"


# tool -> what it takes and what it is allowed to do. Mirrors the ToolSpec
# shape OmniLink's agents already use (typed params + a safety tier), so a
# frame can be validated BEFORE anything executes rather than trusted.
SPECS: Dict[str, Dict[str, Any]] = {
    # ⚠️ `distance` is REQUIRED (owner decision 2026-09-23). A drive with no
    # distance used to pass here and travel the router's 1 m default -- a
    # number nobody said, applied after `_invents_magnitude` had looked.
    "drive_forward":   {"physical": True, "tier": GUARDED,
                        "args": {"distance": float},
                        "required": ("distance",)},
    "drive_to":        {"physical": True, "tier": GUARDED,
                        "args": {"x": float, "y": float},
                        "required": ("x", "y")},
    "turn":            {"physical": True, "tier": GUARDED,
                        "args": {"angle_rad": float},
                        "required": ("angle_rad",)},
    "set_velocity":    {"physical": True, "tier": GUARDED,
                        "args": {"v": float, "w": float},
                        "required": ("v", "w")},
    "stop":            {"physical": True, "tier": SAFE, "args": {}},
    "hold":            {"physical": True, "tier": SAFE,
                        "args": {"release": str}},
    "resume_autonomy": {"physical": True, "tier": GUARDED, "args": {}},
    "reset_to_home":   {"physical": True, "tier": GUARDED, "args": {}},
    "attach_trolley":  {"physical": True, "tier": GUARDED,
                        "args": {"trolley": str, "spot": int}},
    "detach_trolley":  {"physical": True, "tier": GUARDED, "args": {}},
    "pick":            {"physical": True, "tier": GUARDED,
                        "args": {"object": str}},
    "place":           {"physical": True, "tier": GUARDED,
                        "args": {"object": str, "target": str}},
    "open_gripper":    {"physical": True, "tier": GUARDED, "args": {}},
    "close_gripper":   {"physical": True, "tier": GUARDED, "args": {}},
    "wave":            {"physical": True, "tier": SAFE, "args": {}},
    "sit":             {"physical": True, "tier": GUARDED, "args": {}},
    "stand":           {"physical": True, "tier": GUARDED, "args": {}},
    "walk":            {"physical": True, "tier": GUARDED,
                        "args": {"distance": float}},
    "takeoff":         {"physical": True, "tier": GUARDED,
                        "args": {"altitude": float}},
    "land":            {"physical": True, "tier": GUARDED, "args": {}},
    "hover":           {"physical": True, "tier": GUARDED, "args": {}},
    "move_body":       {"physical": True, "tier": GUARDED,
                        "args": {"forward": float, "lateral": float,
                                 "vertical": float}},
    # Read-only. Never gated on interrogatives -- answering a question is
    # exactly what these are for.
    "get_robot_state": {"physical": False, "tier": SAFE, "args": {"kind": str}},
    "clarify":         {"physical": False, "tier": SAFE, "args": {}},
    "decline":         {"physical": False, "tier": SAFE, "args": {}},
    "remember":        {"physical": False, "tier": SAFE, "args": {}},
}

# An utterance that ASKS. Deliberately broad: the cost of treating a command
# as a question is one clarifying reply; the cost of the reverse is a robot
# moving when somebody asked it a question.
_INTERROGATIVE = re.compile(
    r"^\s*(?:so\s+|and\s+|but\s+)?"
    # ⚠️ `do` and `have` lead a question AND an imperative. "do not move",
    # "do a lap of the aisle" and "have the robot drive forward" are
    # orders, and all three were refused as questions. The lookaheads keep
    # the question reading while releasing the imperative one -- and "do
    # not move" mattered most, because its only correct action is a stop.
    r"(what|where|when|why|how|which|who|whose|is|are|was|were|"
    r"do(?!\s+(?:not|n'?t|a|an|the|it|that|this|another|one))|does|"
    r"did|have(?!\s+(?:the|a|an|it|them|him|her|your|my|our))|"
    r"has|had|am|should|could|would|will|can)\b",
    re.IGNORECASE)

# ...except when it is a polite imperative. "can you stop?" is an order.
_POLITE_IMPERATIVE = re.compile(
    r"^\s*(?:so\s+)?(?:can|could|would|will|please)\s+(?:you\s+)?(?:please\s+)?"
    r"(?!tell|show|give|let|remind|explain)",
    re.IGNORECASE)

# ...and a hypothetical is a question however politely it is dressed.
_HYPOTHETICAL = re.compile(
    r"\b(?:in principle|in theory|hypothetically|theoretically|"
    r"if i (?:asked|told|said|wanted)|would you be able|are you able|"
    r"could you ever|is it possible)\b", re.IGNORECASE)

_PROHIBITION = re.compile(
    # ⚠️ `never(?!\s+mind)`: "never mind" is a RETRACTION, not a
    # prohibition. Without the lookahead, "reverse 3 metres. never mind,
    # go forward 1 metre" was refused outright -- a false refusal, which
    # is an order a person gave and a robot ignored. Found 2026-09-21 on
    # the SECOND held-out set, not on the one the rules were written for.
    r"\b(?:don'?t|do not|never(?!\s+mind)|must not|mustn'?t|no longer|stay out of|"
    r"keep out of|steer clear of|avoid|stop\s+\w+ing|"
    # Prohibitions with NO negation particle. langsoak measured the parser
    # failing exactly these while passing "avoid" and "must not": the guard
    # was a list and these were the items nobody added. "under no
    # circumstances drive forward" drove the robot a metre.
    r"under no circumstances|on no account|at no point|in no case|"
    r"refrain from|hold off on|hold fire|desist from|"
    r"don'?t you dare|do not dare)\b", re.IGNORECASE)

_SELF_NEGATING = re.compile(
    r"\bwithout\s+(?:moving|driving|turning|going|rotating|budging)\b|"
    r"\bbut\s+(?:don'?t|do not|never)\b|\bwhile\s+not\s+moving\b",
    re.IGNORECASE)

# Bare pronouns. A frame carrying one of these as a slot value was guessed.
_DEIXIS = {"it", "that", "this", "them", "those", "these", "there", "here",
           "over there", "the other side", "the other one"}

_FORWARD_WORD = re.compile(r"\b(?:forward|forwards|ahead)\b", re.IGNORECASE)
# ⚠️ A sentence that names BOTH directions cannot convict a negative
# distance of a sign error. "drive forward 0.7 metres, then reverse 0.7
# metres" is two correct frames, and checking "forward" against the whole
# sentence refused the second -- the robot stopped 0.7 m from where it was
# told to return to. The gate sees one frame at a time and cannot tell which
# clause produced it, so when both directions are named it abstains and the
# magnitude rules still apply. Found by the harness-comparison pilot,
# 2026-09-22 (compound_04, repeat_01).
# Bare "back" is only a direction as a verb particle: "the back wall" and
# "the back door" must not switch the check off.
_BACKWARD_WORD = re.compile(
    r"\b(?:backward|backwards|reverse|reversing|retreat|rearward|"
    r"rearwards)\b|\bback\s+(?:up|off|out|away)\b|"
    r"\b(?:go|come|move|drive|head|roll|step|get)\s+back\b",
    re.IGNORECASE)

# ── Reported speech. A sentence ABOUT an order is not an order. ──────
#
# "the manual says 'drive forward 2 metres'", "someone shouted 'turn
# left!'", "my colleague asked whether you could reverse 1 metre". A
# matrix clause makes the imperative a thing being DESCRIBED, and both the
# parser and the model reach past it and act. Measured 2026-09-21: 3/3 of
# this family actuated a model, 144/144 actuated the shipped bridge.
#
# Deliberately requires a SUBJECT before the attribution verb, so it does
# not fire on a genuine order that happens to contain one ("tell me your
# status" is an instruction; "he told me your status" is a report).
# Any subject, but only INFLECTED attribution verbs. That distinction is
# what keeps it off genuine imperatives: "tell me when you arrive" and
# "say the word and drive" are orders and use the bare form, while a
# report needs "told", "says", "radioed". Matching the bare forms too
# would refuse both of those sentences, and a false refusal is an order a
# person gave and a robot ignored.
_REPORTED = re.compile(
    r"\b\w+\s+"
    r"(?:says|said|asks|asked|shouts|shouted|tells|told|claims|"
    r"claimed|wrote|writes|reads|mentioned|mentions|suggests|"
    r"suggested|states|stated|radioed|reports|reported|noted|notes|"
    r"indicated|indicates|advised|advises|recommends|recommended|"
    r"insisted|insists|warned|warns)\b",
    re.IGNORECASE)
# ⚠️ THAT IS A LIST, AND A LIST HAS HOLES. "states" was missing until a
# second held-out set caught it — the same structural weakness this rule
# exists to fix one level down. A real solution needs the
# subject-verb-complement shape, not an enumeration of verbs. Recorded
# rather than papered over: expect this to miss attributions nobody
# thought of, and prefer adding a verb here over widening the pattern
# until it starts swallowing genuine imperatives.

# ── Retraction with nothing to replace it. ───────────────────────────
#
# "reverse 2 metres — sorry, ignore that" cancels. "drive forward 3
# metres, scratch that, 1 metre" REPLACES, and must still run. The two
# are told apart by whether anything actionable follows the marker, not
# by which marker it is -- the parser keyed on the phrase and so passed
# one wording of each pair and failed the other.
_CANCEL = re.compile(
    r"\b(?:ignore (?:that|it|those)|forget (?:that|it)|never mind|"
    r"scratch that|disregard (?:that|it)|cancel that|belay that|"
    r"as you were|stand down|strike that)\b", re.IGNORECASE)
_ACTIONABLE_AFTER = re.compile(
    r"\d|\b(?:one|two|three|four|five|six|seven|eight|nine|ten|half|"
    r"quarter|left|right|forward|backward|back|ahead|instead|rather)\b",
    re.IGNORECASE)


# Attribution verbs that are ALSO common warehouse nouns. After a
# determiner these are the noun ("the reports", "the delivery notes",
# "cycle through all states"), and treating them as speech refused a pile
# of ordinary orders. Found by adversarial review, 2026-09-21.
_NOUNY = {"reports", "notes", "states", "claims", "asks", "writes",
          "reads", "warns", "insists"}
# What a reported-speech VERB takes and a noun does not.
_COMPLEMENTISER = re.compile(r"""\s*(?:that\b|to\b|:|["'“‘])""",
                             re.IGNORECASE)
_DETERMINER = {"the", "a", "an", "these", "those", "this", "that", "all",
               "some", "any", "my", "your", "his", "her", "its", "their",
               "our", "no", "both", "each", "every", "which", "what"}


# ── Deferred to an event the robot cannot observe. ──────────────────
#
# "once the shift ends, drive forward 2 metres" is a plan, not an order:
# the trigger has not fired and the robot has no way to know when it
# does. Measured 2026-09-21 -- the parser actuated on this, and the gate
# had no rule for it.
#
# Narrow on purpose. It fires only on a SUBORDINATING time conjunction
# introducing a clause, or an explicit future offset. "turn left and then
# drive forward" is sequencing within one order and must still run, so
# `then` is deliberately absent.
_DEFERRED = re.compile(
    r"\b(?:once|when|whenever|after|as soon as|the moment|"
    r"the next time|if and when)\s+\w+|"
    r"\bin\s+(?:a|an|\d+|one|two|three|five|ten|fifteen|twenty|thirty)\s+"
    r"(?:second|minute|hour|moment)s?\b",
    # ⚠️ BARE TIME ADVERBS ARE NOT HERE, and must not be added back.
    # `later|tomorrow|tonight|afterwards` matched anywhere in the
    # utterance, so "we'll find out later. square her up - 90 degrees
    # right." was refused -- the adverb belonged to the PREVIOUS
    # sentence and the order was in the present. That is a false refusal,
    # and it was the only failed command in the 50-prompt shift demo.
    # A subordinating conjunction defers structurally; an adverb floating
    # somewhere in the paragraph does not.
    re.IGNORECASE)

# ── Self-contradiction inside one utterance. ────────────────────────
#
# "drive forward and also stay exactly where you are" and "turn left and
# right at the same time" cannot both be satisfied. The parser took the
# first verb and ran, which is the ladder's own failure mode.
_CONTRADICTION = re.compile(
    r"\band\s+(?:also\s+)?(?:stay|remain|stand)\s+(?:exactly\s+)?"
    r"(?:where|put|still|in place)\b|"
    r"\bwhile\s+(?:also\s+)?(?:staying|remaining|standing)\s+(?:still|put)\b|"
    r"\b(?:left|right|forward|back|backwards?)\s+and\s+"
    r"(?:left|right|forward|back|backwards?)\s+at\s+the\s+same\s+time\b|"
    r"\bat\s+the\s+same\s+time\s+as\s+(?:staying|stopping)\b",
    re.IGNORECASE)


# ── A magnitude in the frame that is nowhere in the utterance. ──────
#
# The bare word "forward" drives one metre, because the executor
# defaults. Measured 2026-09-21: after every other rule was in place it
# was the ONLY remaining false actuation in a 36-minute run, firing every
# time the case came round.
#
# It is the same defect as "two metres" once driving exactly 1.000 m --
# a silent guess. Refusing an underspecified order is safe; inventing a
# number for it is not, and the invented number is indistinguishable
# downstream from one the operator actually said.
# ⚠️ A LIST OF NUMBER WORDS WITH HOLES REFUSES REAL NUMBERS. "seventy",
# "eighty", "zero" and thirteen to nineteen were missing, so "turn
# seventy degrees left" and "back up fifteen centimetres" were refused as
# invented magnitudes -- whoever produced the frame. Found 2026-09-23.
_HAS_NUMBER = re.compile(
    r"\d|\b(?:zero|nought|one|two|three|four|five|six|seven|eight|nine|ten|"
    r"eleven|twelve|thirteen|fourteen|fifteen|sixteen|seventeen|eighteen|"
    r"nineteen|twenty|thirty|forty|fifty|sixty|seventy|eighty|ninety|"
    r"hundred|dozen|half|quarter|third|tenth|tenths|couple)\b|"
    # "a metre", "an inch" -- the article IS the magnitude.
    r"\b(?:a|an)\s+(?:metre|meter|m|centimetre|centimeter|cm|millimetre|"
    r"millimeter|mm|foot|feet|inch|degree|radian|turn)\b",
    re.IGNORECASE)
# Words that carry their own magnitude without stating one.
_IMPLIED_MAGNITUDE = re.compile(
    r"\b(?:around|about-face|180|home|origin|start|back to|all the way|"
    r"fully|completely|right round)\b", re.IGNORECASE)
_MAGNITUDE_ARGS = ("distance", "angle_rad", "altitude")
_VAGUE_AMOUNT = re.compile(r"\b(?:a\s+(?:little|bit|touch|tad|nudge)|few|slightly)\b", re.I)
_QUANTIFIED_MOTION = re.compile(
    r"\b(?:drive|driving|move|moving|go|going|reverse|reversing|back|"
    r"turn|turning|rotate|rotating|walk|walking|nudge|roll|advance|retreat|"
    r"climb|descend|ascend|fly|takeoff|take off)\b", re.I)
_MEASURED_AMOUNT = re.compile(
    r"(?:(?:" + _HAS_NUMBER.pattern + r")\s*|\b(?:a|an)\s+)"
    r"(?:metres?|meters?|centimetres?|centimeters?|millimetres?|millimeters?|"
    r"m|cm|mm|feet|foot|inches?|degrees?|deg|radians?|rad|turns?)\b", re.I)
_MOTION_CLAUSES = re.compile(
    r"(?<!\d)[.!?;]|[.!?;](?!\d)|\bthen\b|"
    r"\band\s+(?=(?:drive|move|go|reverse|back|turn|rotate|walk)\b)", re.I)


def _invents_magnitude(utterance: str, tool: str, args: Dict[str, Any]) -> bool:
    """True when a frame supplies a number the sentence never mentioned.

    ⚠️ VACUOUS WITHOUT AN UTTERANCE, AND MUST STAY SILENT THERE. "the
    utterance never gave a number" means nothing when there is no
    utterance, and `/tool` is exactly that path: a bare tool call with
    arguments and no sentence.

    Getting this wrong bricked a robot surface. When the rule first
    landed, `check("", [drive_forward {distance: 2}])` returned
    invented_magnitude, so the platform's `/tool` callback refused EVERY
    drive, turn and takeoff with a non-zero magnitude. The e2e test that
    should have caught it had run before the rule existed and stayed
    green. It was found by a second implementation of this gate reading
    the same code -- which is a good argument for having one.
    """
    if not (utterance or "").strip():
        return False
    vals = [args.get(n) for n in _MAGNITUDE_ARGS]
    if not any(isinstance(v, (int, float)) and not isinstance(v, bool) and v
               for v in vals):
        return False
    u = utterance
    # A robot ID, repetition count or another leg's distance cannot give
    # meaning to 'a bit'. An explicit local quantity may clarify it.
    for clause in _MOTION_CLAUSES.split(u):
        if (_QUANTIFIED_MOTION.search(clause) and _VAGUE_AMOUNT.search(clause)
                and not _MEASURED_AMOUNT.search(clause)):
            return True
    return not (_HAS_NUMBER.search(u) or _IMPLIED_MAGNITUDE.search(u))


# A restriction on repeating a completed operation does not prohibit its
# first attempt. Only a complete, explicitly success-qualified clause is
# recognized; other prohibitions remain in the text for the normal checks.
_SUCCESS_REPEAT_LIMIT = re.compile(
    r"\b(?:do not|don't|never)\s+(?:repeat|retry)\s+"
    r"(?:a|an|the|any)\s+(?:already\s+)?(?:successful|completed)\s+"
    r"(?:movement|motion|action|operation|command)\s*(?=[.!?;]|$)", re.I)
_STRAIGHT_ONLY = re.compile(r"\bwithout\s+(?:turning|rotating)\s*(?=[.!?;,]|$)", re.I)
_EXPLICIT_MOTION_REQUEST = re.compile(
    r"(?:^|[.!?;])\s*(?:(?:please|now|then)\s+)*"
    r"(?:drive|move|go|reverse|back|turn|rotate|walk|pick|place|set)\b", re.I)


def _intent_guard_text(utterance: str, tool: str) -> str:
    """Scope two bounded modifiers without deleting other prohibitions."""
    def restriction(match):
        # A standalone ban on repetition is not itself an action request.
        return " " if _EXPLICIT_MOTION_REQUEST.search(utterance[:match.start()]) else match.group()
    text = _SUCCESS_REPEAT_LIMIT.sub(restriction, utterance)
    if tool == 'drive_forward':
        text = _STRAIGHT_ONLY.sub(' ', text)
    return text


# ⚠️ SEQUENCING IS NOT DEFERRAL. "reverse 0.35 m and after that swivel
# 135 degrees" is one order with two steps, and "retry once if the tool is
# unavailable" uses `once` to mean one time. Both were refused as deferred
# (harness-comparison held-out set, 2026-09-23). "after that meeting" and
# "once the shift ends" still defer: only "after that/this" followed by the
# next step, and `once` followed by more/again/if/and/then or the end of a
# clause, are rewritten before the check.
_SEQUENCE_WORDS = (r"turn|drive|go|move|reverse|back|rotate|spin|swivel|pivot|"
                   r"stop|head|advance|travel|come|roll|creep|return|wait|pause|"
                   r"pick|place|put|open|close|lift|walk|fly|land|climb|take|do|"
                   r"make|swing|face")
_NOT_A_TRIGGER = re.compile(
    r"\bafter\s+(?:that|this)\b(?=\s*[,:-]|\s+(?:please\s+)?(?:"
    + _SEQUENCE_WORDS + r")\b)|"
    r"\bonce(?=\s*(?:more|again|if|and|or|then)\b|\s*[,.;!?]|\s*$)",
    re.IGNORECASE)


def _is_deferred(utterance: str) -> bool:
    return bool(_DEFERRED.search(_NOT_A_TRIGGER.sub(" then ", utterance or "")))


def _is_contradictory(utterance: str) -> bool:
    return bool(_CONTRADICTION.search(utterance or ""))


def _is_reported(utterance: str) -> bool:
    """True when the sentence REPORTS an order rather than giving one.

    Two-stage on purpose. The regex finds `<word> <attribution verb>`, and
    then the word before a noun-collidable verb is checked: a determiner
    means the token is a noun and this is not reported speech. A single
    regex cannot express that without variable-length lookbehind, and the
    version that tried refused "collect the reports from bay 3".
    """
    u = utterance or ""
    for m in _REPORTED.finditer(u):
        head = m.group(0).split()
        if len(head) < 2:
            continue
        verb = head[-1].lower()
        # For the noun-collidable tokens, what FOLLOWS decides. A verb
        # reading takes a complementiser -- "the handbook states THAT you
        # should" -- while a noun reading takes a preposition or nothing:
        # "the delivery notes OFF the counter", "the reports FROM bay 3".
        #
        # The determiner is not the discriminator, which is what the first
        # attempt assumed: "the handbook states" and "the reports" both
        # have one, and only the first is speech.
        if verb in _NOUNY and not _COMPLEMENTISER.match(u[m.end():]):
            continue
        # The operator correcting THEIR OWN earlier order is a correction, not
        # a report: "I said reverse 1.2 m a moment ago - ignore that, the real
        # number is 0.6 m" was refused in every arm (held-out v3, 2026-09-24).
        # Only a first-person subject, and only with a correction after it:
        # "I said drive forward 2 metres" alone is still left to this rule.
        if head[0].lower() in ("i", "we") and (
                _CANCEL.search(u[m.end():]) or _CORRECTION.search(u[m.end():])):
            continue
        # An attribution inside a CONDITION that reports no order is a
        # condition, not reported speech: "if the motion tool says it's
        # temporarily unavailable, retry once" was refused (held-out set,
        # 2026-09-23). "If the manual says drive forward, ..." still counts:
        # its complement carries an order.
        if _in_condition(u, m.start()) and \
                not _ORDER_WORD.search(_CLAUSE_END.split(u[m.end():], 1)[0]):
            continue
        return True
    return False


_CONDITION_HEAD = re.compile(
    r"\b(?:if|when|whenever|should|unless|in case)\b", re.IGNORECASE)
_ORDER_WORD = re.compile(
    r"\d|\b(?:drive|turn|reverse|back|forward|ahead|rotate|spin|swivel|pivot|"
    r"go|move|stop|halt|left|right|advance|travel|climb|descend|land|takeoff|"
    r"take off|pick|place|lift|walk|fly|reset|return|metres?|meters?|"
    r"degrees?)\b", re.IGNORECASE)
_CLAUSE_END = re.compile(r"[,.;!?]")


def _in_condition(u: str, at: int) -> bool:
    """True when position `at` sits inside a conditional clause of its sentence."""
    start = max(u.rfind(c, 0, at) for c in ".!?;") + 1
    return bool(_CONDITION_HEAD.search(u[start:at]))


def _is_bare_retraction(utterance: str) -> bool:
    """A cancel marker with nothing actionable after it."""
    m = None
    for m in _CANCEL.finditer(utterance or ""):
        pass
    if m is None:
        return False
    return not _ACTIONABLE_AFTER.search(utterance[m.end():])


def _is_question(utterance: str) -> bool:
    u = (utterance or "").strip()
    if not u:
        return False
    if _HYPOTHETICAL.search(u):
        return True
    if _POLITE_IMPERATIVE.match(u) or _POLITE_MIND.match(u):
        return False
    # "Should your y coordinate be above 0.4 m, drive back 0.5 m; otherwise
    # turn left" is an inverted CONDITION followed by an order, not a question
    # -- it was refused in every arm (held-out v3, 2026-09-24). Only without a
    # question mark, and only when an order follows the condition's comma.
    if _INVERTED_CONDITION.match(u) and not u.endswith("?"):
        return False
    # "roll ahead one metre forty, would you?" is an order with a request
    # tag, and was refused as a question (held-out set, 2026-09-23). Only
    # request tags count: "you drove forward, right?" stays a question.
    tag = _REQUEST_TAG.search(u)
    if tag:
        head = u[:tag.start()].strip()
        return not head or bool(_INTERROGATIVE.match(head))
    return bool(u.endswith("?") or _INTERROGATIVE.match(u))


# "Mind giving the robot a quarter turn to the left?" / "would you mind
# turning left?" ask for an action. "do you mind if I ..." asks permission.
_POLITE_MIND = re.compile(
    r"^\s*(?:(?:would|do)\s+you\s+)?mind\s+(?!if\b)\w+ing\b", re.IGNORECASE)
_INVERTED_CONDITION = re.compile(
    r"^\s*(?:should|were|had)\s+(?:your|the|my|its|it|you)\b[^?]*?[,;]\s*"
    r"(?:then\s+)?(?:please\s+)?(?:" + _SEQUENCE_WORDS + r")\b", re.IGNORECASE)
_CORRECTION = re.compile(
    r"\b(?:correction|actually|instead|i meant|make (?:it|that)|"
    r"the (?:real|right|correct|actual) (?:number|distance|angle|value|one))\b",
    re.IGNORECASE)

_REQUEST_TAG = re.compile(
    r"[,;]\s*(?:(?:would|could|will|can)\s+you(?:\s+please)?|please)\s*\?\s*$",
    re.IGNORECASE)


# ── The bridge's HTTP tool surface, in the gate's own vocabulary ─────
#
# ⚠️ SPECS IS THE PARSER'S FRAME VOCABULARY. The bridge's HTTP tools are
# named differently -- `stop_robot` not `stop`, `hold_until_told` not
# `hold` -- and carry parameters the frames do not, like `wait`. That did
# not matter while only the parser produced frames.
#
# It matters the moment a MODEL produces them, which is the whole point of
# a gate that does not care who wrote the frame. Measured 2026-09-21
# (tests/benchmarks/interpbench): of 8 model false actuations the gate
# appeared to catch, only 3 were caught on meaning. The other 4 were
# "unknown_tool: stop_robot" and "unknown_arg: wait" -- schema drift, not
# judgement. A gate that rejects by accident also rejects what is valid:
# a legitimate stop_robot was refused for the same reason.
#
# So the vocabularies are reconciled HERE, at the boundary, rather than by
# rewriting SPECS and making the parser speak HTTP.
_TOOL_ALIASES = {
    "stop_robot": "stop",
    "hold_until_told": "hold",
    "pause_after_current_task": "hold",
}

# Arguments the bridge accepts that carry no SAFETY meaning, and so are
# dropped before the rails run. An unknown argument is still a rejection,
# because it usually means the caller invented one.
#
# ⚠️ `speed` is deliberately NOT here. It is a real parameter and it is a
# magnitude, so it is declared in SPECS and checked against MAX_SPEED_MPS.
# Stripping it would have made 99 m/s invisible.
_TRANSPORT_ARGS = {"wait", "words", "until_told"}

# Read-only or scheduling tools the bridge exposes and the parser has no
# frame for. Non-physical: they may answer a question, unlike motion.
_PLATFORM_SAFE = {
    "pause_when", "notify_when", "list_pending_intents",
    "estimate_time_remaining", "cancel_pending_intent",
    "set_constraint", "clear_constraint",
}

_ARG_ALIASES = {
    ("set_velocity", "linear"): "v",
    ("set_velocity", "angular"): "w",
}

# Declared so they are KNOWN, not so they are trusted. Each is non-physical:
# scheduling and bookkeeping, which a question is allowed to produce. Arg
# names come from the bridge's own tool definitions (bridge_payload.json).
for _t, _a in {
    "pause_when": {"condition": object, "count": object, "leg": object},
    "notify_when": {"condition": object, "count": object, "leg": object,
                    "message": object, "pause_first": object},
    "list_pending_intents": {},
    "estimate_time_remaining": {},
    "cancel_pending_intent": {"id": object},
    "set_constraint": {"minutes": object, "rule": object},
    "clear_constraint": {"id": object},
}.items():
    SPECS.setdefault(_t, {"physical": False, "tier": "safe", "args": _a})

# Parameters the bridge's physical tools really take, beyond what the
# parser's frames carry. `speed` and `wait` are stripped by normalise();
# these are the ones with meaning.
SPECS["drive_forward"]["args"].setdefault("speed", float)


# ── Registration: the bridge's own schema, not a second copy of it ───
#
# ⚠️ EVERYTHING ABOVE THIS LINE IS HAND-WRITTEN, AND THAT WAS THE BUG.
#
# SPECS started as the parser's frame vocabulary, which is a closed set and
# correct for that job. Then the gate was put in front of the bridges' HTTP
# tool surface, which is an OPEN set, and nothing reconciled the two. On
# 2026-09-21 the drift measured:
#
#   arm        19 tools registered, 10 absent from SPECS
#   mobile     11 tools registered,  1 absent
#   quadruped   6 tools registered,  0 absent
#
# and it failed in two opposite directions at once.
#
#   A tool SPECS knows, with arg names it does not  ->  falsely refused.
#       `place{xyz}` and `attach_trolley{def}` are the bridges' real
#       parameters; SPECS declared object/target and trolley/spot. Both
#       features were unreachable over the platform's callback.
#
#   A tool SPECS does not know at all  ->  NOT GATED.
#       check() emits `unknown_tool` and skips every other rule, and all
#       five call sites filter `unknown_tool` out -- correctly, since the
#       caller already resolved the tool. Net effect: `grasp`,
#       `set_tcp_target`, `set_joint_positions`, `set_gripper_width`,
#       `run_learned_skill`, `goto_waypoint` and `set_yaw` reached a motor
#       with no schema, magnitude or deixis check whatsoever. The arm's
#       most physical verbs were its least protected.
#
# The fix is not a bigger table. The bridge ALREADY publishes a correct
# JSON schema for every tool, at registration time, in `Tool.parameters`.
# `register_tools()` reads it. A tool the bridge registers is a tool the
# gate knows, with the arg names the bridge actually uses, by construction
# rather than by maintenance.
#
# ⚠️ The RAILS stay declared above, in this file. OmniLink ships a
# TypeScript port of this module and the only thing binding the two is the
# generated fixture tests/benchmarks/gate_parity/cases.json. A safety
# constant that moves into a bridge is a safety constant the fixture stops
# freezing. Schemas come from the registry; limits do not.

_JSON_TO_PY = {
    "number": float, "integer": int, "string": str,
    "boolean": bool, "array": list, "object": dict,
}


def is_physical(tool_name: str) -> bool:
    """Can this tool move something?

    ⚠️ ASK THE REGISTRY FIRST. The old answer was a substring match on the
    tool's NAME, copy-pasted into five files, and it is wrong in both
    directions: `get_drive_status` contains "drive" and is read-only,
    `activate_sprayer` contains nothing and is not. Now that bridges
    register their tools with `physical=`, the name heuristic is only the
    last resort for a tool nobody declared.
    """
    spec = SPECS.get((_TOOL_ALIASES.get(tool_name, tool_name)))
    if spec is not None:
        return bool(spec.get("physical"))
    low = (tool_name or "").lower()
    return any(h in low for h in _PHYSICAL_NAME_HINTS)


# Only reached for a tool no bridge registered and no table declares --
# a runtime-named arm skill, or a bare clone with no relay.
_PHYSICAL_NAME_HINTS = (
    "drive", "turn", "set_velocity", "stop", "reset_to_home",
    "resume_autonomy", "move", "walk", "takeoff", "land", "hover",
    "pick", "place", "grip", "grasp", "release", "sit", "stand",
    "joint", "tcp", "trolley",
)


def reject_toolcall(tool_name: str, args: Dict[str, Any],
                    utterance: str = "",
                    surface: Optional[str] = None) -> Optional[str]:
    """Vet ONE bare tool call. None means allow; a string is the reason.

    THE CANONICAL IMPLEMENTATION. It used to be copy-pasted into five
    files -- `bridge_base`, `relay`, and the mobile / arm / quadruped
    controllers -- which is how commit b977772b0 came to add a gate to the
    base and miss all four bridges. A safety check that has to be applied
    in five places is a safety check that will be applied in four.

    ⚠️ `unknown_tool` is dropped. The gate does not police tool EXISTENCE
    -- the bridge's registry already did, and overruling it would refuse
    the arm's learned verbs, which are named at runtime. Every other rule
    still applies. Note that this is exactly why registration matters: a
    tool the gate has never heard of is not merely unknown, it is
    UNCHECKED, and until 2026-09-21 that included most of the arm.
    """
    rej = [r for r in check(utterance or "",
                            [{"tool": tool_name, "args": dict(args or {})}],
                            surface=surface)
           if r.rule != "unknown_tool"]
    return f"{rej[0].rule}: {rej[0].detail}" if rej else None


def _args_from_schema(parameters: Dict[str, Any]) -> Dict[str, Any]:
    """{name: python type} from a JSON-schema parameter block."""
    props = (parameters or {}).get("properties") or {}
    out: Dict[str, Any] = {}
    for name, spec in props.items():
        if not isinstance(spec, dict):
            continue
        t = spec.get("type")
        if isinstance(t, list):            # ["number", "null"] and friends
            t = next((x for x in t if x != "null"), None)
        out[name] = _JSON_TO_PY.get(t, object)
    return out


def register_tools(tools: Iterable[Any], surface: Optional[str] = None) -> List[str]:
    """Teach the gate the tools a bridge actually serves. Returns names added.

    Merges rather than replaces: a tool already in SPECS keeps its
    hand-declared args (the parser emits frames in that vocabulary) and
    GAINS the bridge's, so `place` accepts both `object`/`target` from the
    parser and `xyz` from the arm. `required` is only taken from the
    schema for tools SPECS did not already know -- tightening an existing
    tool's requirements from a bridge schema would refuse parser frames
    that are valid today.

    Idempotent, and safe to call from several bridges in one process.
    """
    added: List[str] = []
    for tool in tools or ():
        name = getattr(tool, "name", None)
        if not name:
            continue
        schema_args = _args_from_schema(getattr(tool, "parameters", {}) or {})
        physical = bool(getattr(tool, "physical", False))
        existing = SPECS.get(name)
        if existing is None:
            req = tuple((getattr(tool, "parameters", {}) or {}).get("required") or ())
            SPECS[name] = {
                "physical": physical,
                # A newly-registered physical tool is GUARDED, never SAFE.
                # SAFE is the de-escalation carve-out -- it exempts a tool
                # from the intent rules -- and that exemption is only ever
                # granted by hand, above, to tools that REDUCE what the
                # robot is doing. Inheriting it from a registry would let a
                # bridge name a tool `stop_and_fling` into the exemption.
                "tier": GUARDED if physical else SAFE,
                "args": dict(schema_args),
                "required": req,
                "surface": surface,
                # Every surface this tool has been registered for. A tool
                # served by two classes cannot be described by one value,
                # and pretending otherwise made the rail depend on
                # registration order.
                "surfaces": {surface} if surface else set(),
            }
            added.append(name)
        else:
            existing.setdefault("args", {})
            for k, v in schema_args.items():
                existing["args"].setdefault(k, v)
            # A tool the bridge declares physical is physical, even if the
            # hand-written entry said otherwise. The reverse is NOT true:
            # nothing registered may downgrade a tool the table calls
            # physical, or a bridge could opt its own actuator out.
            if physical:
                existing["physical"] = True
            if surface:
                existing.setdefault("surfaces", set()).add(surface)
                if existing.get("surface") is None:
                    existing["surface"] = surface
    return added


def normalise(tool: str, args: Dict[str, Any]) -> Tuple[str, Dict[str, Any]]:
    """Map a bridge-surface tool call into the vocabulary SPECS declares.

    Pure and total: an unrecognised tool passes through unchanged so it
    still trips `unknown_tool` on purpose rather than by accident.
    """
    canon = _TOOL_ALIASES.get(tool, tool)
    out: Dict[str, Any] = {}
    for k, v in (args or {}).items():
        if k in _TRANSPORT_ARGS:
            continue
        out[_ARG_ALIASES.get((canon, k), k)] = v
    return canon, out


def _frame_parts(frame: Any):
    """Accept a Frame dataclass or a plain {"tool": ..., "args": {...}}."""
    if isinstance(frame, dict):
        tool, args = str(frame.get("tool", "")), dict(frame.get("args") or {})
    else:
        tool = str(getattr(frame, "tool", ""))
        args = dict(getattr(frame, "args", {}) or {})
    return normalise(tool, args)


def check(utterance: str, frames: Sequence[Any],
          authorized: Iterable[str] = (),
          surface: Optional[str] = None) -> List[Rejection]:
    """Every reason these frames may not run for this utterance.

    An empty list means dispatch. Nothing here looks at WHO produced the
    frames, which is the entire point.

    `surface` is the robot class the CALLER serves ("mobile", "arm",
    "quadruped", "drone"). It selects the magnitude rail where two classes
    share a tool -- `move_body` means a body shift on a quadruped and a
    flight on a drone, and 40 m is routine for one and absurd for the
    other. Omit it and the rail falls back to the surface recorded on the
    tool's spec, which is right for every tool only ONE class serves; a
    tool two or more classes registered discards the recorded surface and
    takes the ground / body-shift rail. ⚠️ Not "the strictest rail" as a
    blanket rule -- see §6 below and GATE_COVERAGE.md §1.
    """
    out: List[Rejection] = []
    u = utterance or ""
    question = _is_question(u)
    reported = _is_reported(u)
    retracted = _is_bare_retraction(u)
    deferred = _is_deferred(u)
    contradictory = _is_contradictory(u)
    ok_tier = set(authorized)
    seen_motion = []

    for frame in frames:
        tool, args = _frame_parts(frame)
        spec = SPECS.get(tool)
        if spec is None:
            out.append(Rejection(tool, "unknown_tool",
                                 "no schema declares this tool"))
            continue
        physical = bool(spec.get("physical"))
        guard_text = _intent_guard_text(u, tool)
        prohibited = bool(_PROHIBITION.search(guard_text))
        negating = bool(_SELF_NEGATING.search(guard_text))
        if physical and spec.get('tier') != SAFE and _SUCCESS_REPEAT_LIMIT.search(u):
            if seen_motion and (tool, args) == seen_motion[-1]:
                out.append(Rejection(tool, 'duplicate_motion',
                                     'a retry contingent on failure cannot be an unconditional duplicate action'))
            seen_motion.append((tool, args))

        # 1-3. What the utterance was, versus what the frame wants to do.
        if physical and question:
            out.append(Rejection(tool, "interrogative",
                                 "the utterance asks a question; a question "
                                 "may not move a robot"))
        # ⚠️ DE-ESCALATING TOOLS ARE EXEMPT FROM THE INTENT RULES.
        #
        # `stop` and `hold` are tier SAFE: they REDUCE what the robot is
        # doing. Blocking one is strictly worse than allowing one, because
        # the failure modes are not symmetric -- a spurious stop costs a
        # pause, a refused stop costs whatever the robot was about to hit.
        #
        # Treating every physical tool alike made the gate refuse the three
        # most ordinary ways a person says stop -- "stop moving", "stop
        # driving", "stop turning" -- because `stop\s+\w+ing` read them as
        # prohibitions. It also refused "stand down" (a _CANCEL marker that
        # IS the halt order), "never mind, stop", "do not move" (whose only
        # correct action is a stop), and every zero-argument tool after a
        # cancel marker. Found by adversarial review, 227 tested pairs,
        # 2026-09-21.
        #
        # The magnitude, schema and deixis rules below still apply to these
        # tools. Only the three INTENT rules are lifted, and only for the
        # direction of travel that makes a robot safer.
        de_escalating = spec.get("tier") == SAFE
        if physical and reported and not de_escalating:
            out.append(Rejection(tool, "reported_speech",
                                 "the utterance reports an order rather "
                                 "than giving one"))
        if physical and deferred and not de_escalating:
            out.append(Rejection(tool, "deferred",
                                 "the order waits on an event that has not "
                                 "happened"))
        if physical and _invents_magnitude(u, tool, args):
            out.append(Rejection(tool, "invented_magnitude",
                                 "the frame carries a distance or angle the "
                                 "utterance never gave"))
        if physical and contradictory:
            out.append(Rejection(tool, "contradiction",
                                 "the utterance asks for two things that "
                                 "cannot both be done"))
        if physical and retracted and not de_escalating:
            out.append(Rejection(tool, "retracted",
                                 "the order was withdrawn and nothing "
                                 "replaced it"))
        if physical and prohibited and not de_escalating:
            out.append(Rejection(tool, "prohibition",
                                 "the utterance forbids an action rather "
                                 "than ordering one"))
        # A stop is what "without moving" asks for, so it cannot negate it.
        # Refusing one failed "report your x and y without moving" whenever
        # the plan led with a harmless stop (harness-comparison pilot,
        # 2026-09-22, state_04).
        if physical and negating and not de_escalating:
            out.append(Rejection(tool, "self_negating",
                                 "the utterance asks to act and not act "
                                 "at once"))

        # 6. Schema: declared args only, right types, finite numbers.
        required = spec.get("required", ())
        declared = spec.get("args", {})
        for name in required:
            if args.get(name) is None:
                out.append(Rejection(tool, "missing_arg",
                                     f"{name} is required and absent"))
        for name, value in args.items():
            want = declared.get(name)
            if want is None:
                out.append(Rejection(tool, "unknown_arg",
                                     f"{name} is not a parameter of {tool}"))
                continue
            if want is float and isinstance(value, bool):
                out.append(Rejection(tool, "bad_type",
                                     f"{name} must be a number"))
            elif want is float and not isinstance(value, (int, float)):
                out.append(Rejection(tool, "bad_type",
                                     f"{name} must be a number, got "
                                     f"{type(value).__name__}"))
            elif want is float and not math.isfinite(float(value)):
                out.append(Rejection(tool, "not_finite",
                                     f"{name} is {value}"))
            elif want is int and not isinstance(value, int):
                out.append(Rejection(tool, "bad_type",
                                     f"{name} must be a whole number"))
            elif want is str and not isinstance(value, str):
                out.append(Rejection(tool, "bad_type",
                                     f"{name} must be text"))

        # 4-5. Magnitudes, and a direction that agrees with its sign.
        # ⚠️ ONLY `distance` belongs on the ground-distance rail. `altitude`
        # and the move_body axes were here too, so a 100 m drone climb was
        # refused for exceeding a 50 m FLOOR distance, and the message named
        # the wrong rail. Each quantity now has a rail of its own scale,
        # below.
        for name in ("distance",):
            v = args.get(name)
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                if abs(v) > MAX_DISTANCE_M:
                    out.append(Rejection(tool, "implausible",
                                         f"{name}={v} exceeds the "
                                         f"{MAX_DISTANCE_M:.0f} m rail"))
                if (name == "distance" and v < 0 and _FORWARD_WORD.search(u)
                        and not _BACKWARD_WORD.search(u)):
                    out.append(Rejection(tool, "sign_conflict",
                                         "the utterance says forward and the "
                                         "distance is negative"))
        # `speed` is a real parameter of the bridge's drive_forward, so it
        # cannot be refused as invented -- but an unbounded one is worse
        # than an unknown one. 99 m/s used to be caught by accident, as
        # `unknown_arg`; it is now caught on the magnitude, which is the
        # reason that survives the parameter being legitimate.
        # ⚠️ RAIL THE QUANTITY, NOT THE NAME. This checked only the literal
        # argument `speed`, so `set_velocity {v: 40}` -- the actual velocity
        # channel -- passed unrailed, as did `{linear: 40}` and `{w: 500}`.
        # A rail that guards one spelling of a number is not a rail.
        for _n in ("speed", "v", "linear"):
            _val = args.get(_n)
            if isinstance(_val, (int, float)) and not isinstance(_val, bool):
                if abs(_val) > MAX_SPEED_MPS:
                    out.append(Rejection(tool, "implausible",
                                         f"{_n}={_val} exceeds the "
                                         f"{MAX_SPEED_MPS:.1f} m/s rail"))
        for _n in ("w", "angular"):
            _val = args.get(_n)
            if isinstance(_val, (int, float)) and not isinstance(_val, bool):
                if abs(_val) > MAX_YAW_RATE_RPS:
                    out.append(Rejection(tool, "implausible",
                                         f"{_n}={_val} exceeds the "
                                         f"{MAX_YAW_RATE_RPS:.1f} rad/s rail"))
        # Altitude is not a floor distance and must not share its rail: 49 m
        # of climb passed the 50 m ground rail as if it were a short drive.
        _alt = args.get("altitude")
        if isinstance(_alt, (int, float)) and not isinstance(_alt, bool):
            if abs(_alt) > MAX_ALTITUDE_M:
                out.append(Rejection(tool, "implausible",
                                     f"altitude={_alt} exceeds the "
                                     f"{MAX_ALTITUDE_M:.0f} m rail"))
        # A body shift is centimetres-to-metres, never tens of metres --
        # for a QUADRUPED. `move_body` is shared with the drone, where the
        # same three axes mean flying, and a drone that climbs 40 m has not
        # done anything implausible.
        #
        # ⚠️ THIS RAIL WAS SURFACE-BLIND AND REFUSED EVERY REAL DRONE CLIMB.
        # Measured 2026-09-21 the moment the Mavic was given a gated /tool:
        # "climb 40 metres" -> `vertical=40.0 exceeds the 1.0 m body-shift
        # rail`. The rail was right about a quadruped and wrong about the
        # only other robot that uses the tool, which is what a global
        # constant named after one robot class does when a second one
        # arrives. Its sibling had the same bug in reverse and is recorded
        # above: `altitude` used to share the 50 m FLOOR-distance rail, so
        # a 100 m climb was refused against a rail for driving.
        #
        # A drone's vertical is an altitude change and is railed as one; a
        # drone's horizontal translation is a flight, not a shift, so it
        # gets the ground-distance rail rather than the body-shift one.
        # ⚠️ THE CALLER'S SURFACE WINS, AND IT HAS TO.
        #
        # `move_body` is served by BOTH the drone and the quadruped, with
        # the same three argument names meaning entirely different things.
        # Recording one surface on the spec made the rail depend on which
        # bridge registered first: register the drone before the quadruped
        # and a 40 m "body shift" was allowed; reverse the order and every
        # real drone climb was refused. An order-dependent safety property
        # is worse than the surface-blind rail it replaced, because it
        # looks correct in whichever order you happen to test.
        #
        # So the surface is an ARGUMENT. A bridge knows what it is; the
        # spec's recorded surface is only the fallback for a caller that
        # did not say, and `takeoff`/`land`/`hover` are unambiguous enough
        # to name themselves.
        # The caller's surface wins. When the caller does not say, fall
        # back to the spec's -- but ONLY if the tool belongs to exactly
        # one surface.
        #
        # ⚠️ A SHARED TOOL WITH NO CALLER SURFACE TAKES THE STRICTEST RAIL.
        # `move_body` is served by the drone and the quadruped, and the
        # reference `bridge_base` handler passes no surface, so an external
        # bridge copying it would otherwise inherit whichever class
        # registered first -- letting a 40 m "body shift" through. Guessing
        # air is the unsafe guess: it raises a 1 m limit to 120 m. Guessing
        # ground is the safe one, and it is wrong only in the direction
        # that refuses a legitimate climb, which is visible and fixable by
        # passing `surface=`.
        _surfaces = spec.get("surfaces") or set()
        _spec_surface = (next(iter(_surfaces)) if len(_surfaces) == 1
                         else (spec.get("surface") or ""))
        if not surface and len(_surfaces) > 1:
            _spec_surface = ""              # ambiguous: take the ground rail
        _surface = surface or _spec_surface
        _is_air = _surface == "drone" or tool in ("takeoff", "land", "hover")
        for _n in ("forward", "lateral", "vertical"):
            _val = args.get(_n)
            if not isinstance(_val, (int, float)) or isinstance(_val, bool):
                continue
            if _is_air:
                _rail, _what = ((MAX_ALTITUDE_M, f"{MAX_ALTITUDE_M:.0f} m altitude")
                                if _n == "vertical"
                                else (MAX_DISTANCE_M, f"{MAX_DISTANCE_M:.0f} m"))
            else:
                _rail, _what = (MAX_BODY_SHIFT_M,
                                f"{MAX_BODY_SHIFT_M:.1f} m body-shift")
            if abs(_val) > _rail:
                out.append(Rejection(tool, "implausible",
                                     f"{_n}={_val} exceeds the {_what} rail"))
        angle = args.get("angle_rad")
        if isinstance(angle, (int, float)) and not isinstance(angle, bool):
            if abs(angle) > MAX_ANGLE_RAD:
                out.append(Rejection(tool, "implausible",
                                     f"angle_rad={angle} is more than four turns"))

        # 7. A slot that is still a pronoun was guessed, not resolved.
        for name, value in args.items():
            if isinstance(value, str) and value.strip().lower() in _DEIXIS:
                out.append(Rejection(tool, "unresolved_referent",
                                     f"{name}={value!r} was never resolved"))

        # 8. Destructive tools need an explicit token.
        if spec.get("tier") == CONFIRM and tool not in ok_tier:
            out.append(Rejection(tool, "needs_authorization",
                                 "this tool requires explicit authorization"))

    return out
