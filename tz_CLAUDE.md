# CLAUDE.md

**AI Development Guide for tropo-core**

This file is the **root of all Claude assistance**. Read this FIRST every session.

---

# Part I: Orientation

## Project Identity

**tropo-core** = Core network process for Tropo mesh networking

| Aspect | Details |
|--------|---------|
| **Language** | Rust |
| **Architecture** | Network Stack (L1-L4) + REDS + Extensions + Gateway |
| **Purpose** | Mesh routing, distributed state, mission logic |
| **Audience** | T1 developers only (T2/T3 connect via Gateway) |

**Current State:** See STATUS.md for implementation progress.

---

## Architecture Overview

### System Structure

```
tropo-core/
├── config/        # Centralized configuration (shape-driven design)
├── network/       # L1-L4 mesh routing stack
├── platform/      # Vehicle abstraction (MAVLink, mock)
├── state/         # Distributed State Layer
│   ├── timekeeper/    # HLC synchronization
│   ├── reds/          # CRDTs
│   └── evaluator/     # State Evaluator (derives MeshMode)
├── runtime/       # Runtime + State interface
│   ├── extension/     # Extension trait, lifecycle
│   └── state/         # State interface implementation
├── extensions/    # Mission-configurable extensions
│   ├── localization/
│   ├── coordination/
│   └── bdll/
├── gateway/       # External app interface
│   ├── ipc/           # ZeroMQ transport
│   └── api/           # Protobuf definitions
└── bin/           # Entry point (receives FDs from sentinel)
```

### Layer Responsibilities

| Layer | Responsibility |
|-------|----------------|
| **L1** TRX Driver | Radio abstraction (USB CDC to TickingT firmware) |
| **L2** Coordinator | Multi-radio management, frame handling |
| **L3** Router | AODV mesh routing, path discovery |
| **L4** Messenger | Reliable delivery, flow control |
| **REDS** | CRDT tables with LWW merge |
| **Runtime** | Extension lifecycle, State interface |
| **Gateway** | ZeroMQ pub/sub with capability-based access |

### Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| **Extensions are sync** | Deterministic execution, simple testing, no race conditions |
| **State is the boundary** | Extensions interact through State, never REDS directly |
| **Single-writer for REDS** | Each node writes only its own entries; conflicts impossible |
| **Gateway in-process** | No IPC overhead for internal access |
| **SE calls cold-path only** | Per-packet crypto uses pre-derived session keys |

### The Central Abstraction

mesh-core revolves around MeshNode (messaging). tropo-core revolves around **State** (distributed shared state). The network exists to synchronize State, not as the primary interface. This is not a refactor—it's a different architecture.

---

## Reading Guide

### Every Session

1. **Read this file** - Quality framework, methodology, architecture
2. **Read STATUS.md** - Current state, what exists, what's planned
3. **Read relevant spec** - Architecture documents for your task

### Reference Documents

| Need to understand... | Read... |
|-----------------------|---------|
| tropo-core architecture | `../mesh-docs-restricted/_tropo-rework/SYSTEM_ARCHITECTURE.md` §18-24 |
| Distributed state (REDS) | `../mesh-docs-restricted/_tropo-rework/DISTRIBUTED_STATE.md` |
| Security model | `../mesh-docs-restricted/_tropo-rework/SECURITY_ARCHITECTURE.md` |
| Technical terms | `../mesh-docs-restricted/_tropo-rework/GLOSSARY.md` |
| How to run/test binary | `docs/DEVELOPMENT.md` |

### Internal Boundary Documentation

| Boundary | Document |
|----------|----------|
| State ↔ REDS | `docs/STATE_REDS_BOUNDARY.md` |
| Network ↔ State | `docs/NETWORK_STATE_BOUNDARY.md` |
| Architecture reform | `docs/ARCHITECTURE_REFORM.md` |

**Do not confuse architecture documentation with implementation status.** The `_tropo-rework` docs define the target; STATUS.md shows what exists.

---

# Part II: Quality Framework

## The Three Quality Pillars

All quality requirements derive from three pillars. Use these to evaluate any design decision.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           QUALITY PILLARS                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  STRUCTURAL              BEHAVIORAL              COGNITIVE                   │
│  (How code is organized) (How code executes)     (How code reads)           │
│                                                                              │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │ Interface       │    │ Correctness     │    │ Predictability  │         │
│  │ Consistency     │    │ (spec-faithful) │    │ (guess from one)│         │
│  │                 │    │                 │    │                 │         │
│  │ Boundary        │    │ Safety          │    │ Discoverability │         │
│  │ Clarity         │    │ (invalid states │    │ (find what you  │         │
│  │                 │    │  impossible)    │    │  need)          │         │
│  │ Composability   │    │                 │    │                 │         │
│  │ (combine without│    │ Determinism     │    │ Local Reasoning │         │
│  │  surprises)     │    │ (same in = out) │    │ (understand     │         │
│  │                 │    │                 │    │  without context)│         │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘         │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Pillar 1: Structural Quality

**Question:** Is the code organized so components compose correctly?

| Dimension | Good | Bad |
|-----------|------|-----|
| **Interface Consistency** | All bridges: `tick() → Result<(Stats, Vec<Action>), Error>` | 6 different tick() patterns |
| **Boundary Clarity** | State owns reads; Orchestrator owns dispatch | Dispatch in 3 different places |
| **Composability** | Traits with clear contracts | Interior mutability mixed with `&mut self` |

### Pillar 2: Behavioral Quality

**Question:** Does the code execute correctly and safely?

| Dimension | Good | Bad |
|-----------|------|-----|
| **Correctness** | Implementation traceable to spec section | "It works" without spec reference |
| **Safety** | Enums for state machines | Booleans with invalid combinations |
| **Determinism** | Sync tick with snapshots | Async reads mid-computation |

### Pillar 3: Cognitive Quality

**Question:** Can developers understand and navigate the code?

| Dimension | Good | Bad |
|-----------|------|-----|
| **Predictability** | Consistent naming: `tick_*`, `drain_*` | Mixed: `tick`, `process_*`, `poll_*` |
| **Discoverability** | One tick() entry point | 8 tick() variants to understand |
| **Local Reasoning** | Function does one thing | `process_io()` secretly ticks router |

---

## Design Principles

These principles implement the quality pillars. They are non-negotiable.

| Principle | Meaning | Pillar |
|-----------|---------|--------|
| **Pit of Success** | Right thing = easy thing; wrong thing = hard | Behavioral |
| **Fail Fast** | Catch errors at compile time, not runtime | Behavioral |
| **Single Source of Truth** | Encode knowledge in types, not comments | Structural |
| **Consistency Over Justification** | One pattern beats optimal per context | Structural |
| **Type-Encoded State** | Use enums for state machines, not booleans | Behavioral |
| **No Fiction Values** | Uninitialized = unusable, not "reasonable default" | Behavioral |
| **Atomic Operations** | Multi-step checks under single lock | Behavioral |
| **Shape Over Validation** | Design types where wrong states are unrepresentable | Structural |
| **Composable Outputs** | Uniform return types that combine with `extend()` | Structural |
| **Isolated Mutations** | Each state has exactly one writer | Behavioral |
| **Local Reasoning** | Understand a function without the whole system | Cognitive |
| **No Error Dead Ends** | Return types must carry failure information | Structural |

### The Architecture Test

> "A new developer, who has never read the documentation, should produce correct code by following compiler errors."

If a rule needs documentation to be followed, the architecture isn't guiding strongly enough. Invest in structure that enforces the rule.

### The Abstraction Test

> "Does this abstraction make incorrect usage harder?"

If yes, embrace it. If it just "looks cleaner" but allows the same mistakes, reject it.

---

## Type Safety Requirements

Prefer compile-time enforcement over runtime checks. The compiler is your ally.

### State Machines: Enums Over Booleans

Booleans allow invalid combinations. Enums make invalid states unrepresentable.

**Bad** (implicit state, invalid combinations possible):
```rust
struct Link {
    is_up: bool,
    is_initialized: bool,
    quality: u8,  // What does this mean if is_up = false?
}
```

**Good** (explicit phases, invalid states unrepresentable):
```rust
enum LinkPhase {
    Uninitialized,                        // Cannot be used
    Up { quality: u8, credits: u8 },      // Usable
    Down { since: Instant },              // Not usable
}
```

The compiler prevents misuse: you can't access `quality` on an `Uninitialized` link because the field doesn't exist in that variant.

### No Fiction Values

Never assign "reasonable defaults" to uninitialized state. Fiction values hide bugs.

**Bad:**
```rust
fn new() -> Link {
    Link { quality: 50, is_up: false, ... }  // Fiction! We haven't heard from firmware
}
```

**Good:**
```rust
fn new() -> Link {
    Link { phase: LinkPhase::Uninitialized }  // Honest: we don't know yet
}
```

With fiction values, code that uses quality before initialization won't crash—it will silently misbehave with quality=50. With explicit Uninitialized state, the type system forces you to handle the "not yet known" case.

### Atomic Operations in Types

If two operations must happen together, make it impossible to do them separately.

**Bad** (caller can forget the second step):
```rust
fn check_credits(&self) -> bool { ... }
fn decrement_credits(&mut self) { ... }
```

**Good** (single operation, impossible to misuse):
```rust
fn reserve_credit(&mut self) -> Result<(), NoCredits> {
    // Check AND decrement atomically
}
```

### Wiring Obligations

When a factory method returns multiple values that MUST ALL be consumed for the system to work correctly, the return type must enforce consumption. Silently dropping a channel receiver, callback, or wiring connection is a class of bug that produces zero symptoms until integration testing.

**The Wiring Test:**

> "If a developer ignores one return value from this factory, does the system fail silently?"

If yes, wrap the return value in `WiringObligation<T>`. This type:
- Is `#[must_use]` — compiler warns if the return value is ignored
- Panics on drop if not consumed — catches the bug at startup, not in production

```rust
// BAD: Caller can silently drop trigger_rx
let (dispatcher, trigger_rx, batched_rx) = TriggerDispatcher::with_channels(clock);
// trigger_rx dropped here — no compiler error, no runtime error, system silently broken

// GOOD: Caller MUST consume or the program panics
let (dispatcher, trigger_rx, batched_rx) = TriggerDispatcher::with_channels(clock);
let trigger_rx = trigger_rx.consume();  // Explicit consumption
// If this line is missing, WiringObligation panics on drop
```

**When NOT to use WiringObligation:** Optional wiring (e.g., `with_platform()` where `None` is valid) doesn't need it. Only use for wiring where dropping silently breaks invariants.

### Shape-Driven Design

When designing data structures, configuration, or APIs, ask:

> **"What is the shape that makes wrong states unrepresentable and right states easy?"**

This shifts focus from **validation** (checking after construction) to **design** (making invalid construction impossible).

| Problem | Validation (Weak) | Shape (Strong) |
|---------|-------------------|----------------|
| Quality must be 0-100 | `assert!(q <= 100)` at runtime | `Percent` newtype that enforces range |
| Timeout can't be zero | Check in constructor | `NonZeroU8` type |
| Config values must correlate | Document the relationship | Derive one from another |
| Some fields only valid together | Runtime checks | Enum variants with associated data |

**Anti-pattern:** Adding validation to fix a bad shape. If you're writing `if config.x < config.y { panic!() }`, the shape is wrong—`x` and `y` should probably be one value or derived.

**Litmus test:** Can a caller construct an invalid instance? If yes, redesign the type.

### Outcomes vs Errors

**Outcomes** are valid results. **Errors** are actual failures. Don't conflate them.

**Bad** (treats policy-drop as error):
```rust
pub async fn send(...) -> Result<(), NetworkError>;
// where NetworkError::Dropped exists - but dropping is normal with QoS!
```

**Good** (separates outcomes from errors):
```rust
pub async fn send(...) -> Result<SendOutcome, CoreError>;
// SendOutcome::Dropped is valid outcome, CoreError is actual failure
```

Why this matters:
- Caller must handle both `Queued` and `Dropped` — compiler enforces it
- "Dropped" is not exceptional — it's expected with QoS policies
- Errors remain errors — actual failures that need attention

| Situation | Model As | Why |
|-----------|----------|-----|
| Message queued | `SendOutcome::Queued` | Success |
| Message dropped by QoS | `SendOutcome::Dropped` | Valid outcome, caller decides to retry |
| No route exists | `CoreError::NoRoute` | Actual failure |
| Payload too large | `CoreError::PayloadTooLarge` | Actual failure |

### Type Naming Convention

Consistent naming makes types discoverable and self-documenting.

| Category | Pattern | Example |
|----------|---------|---------|
| Outcomes | `<Op>Outcome` | `SendOutcome`, `CreateOutcome` |
| Errors | `<Boundary>Error` | `CoreError`, `ServiceError`, `ExtensionError` |
| Status | `<Thing>Status` | `LinkStatus`, `SessionStatus` |
| Info | `<Thing>Info` | `RouteInfo`, `PeerInfo`, `NeighborInfo` |
| Events | `<Domain>Event` | `ServiceEvent`, `LinkEvent` |

**Error hierarchy:** Each boundary wraps inner errors.

```rust
// Core boundary
pub enum CoreError { NoRoute(PeerId), PayloadTooLarge, NotRunning, ... }

// Service boundary (wraps core)
pub enum ServiceError { Core(CoreError), NoSession(PeerId), ... }

// Extension boundary (wraps service)
pub enum ExtensionError { Service(ServiceError), InvalidState, ... }
```

### Composable Outputs

Components that produce multiple items should return uniform types that combine naturally.

**Bad** (grab-bag struct, different producers fill different fields):
```rust
struct ServiceOutput {
    routes: Vec<RouteUpdate>,      // Only routing fills
    sync_records: Vec<SyncRecord>, // Only sync fills
    links: Vec<LinkStatusChange>,  // Only link fills
}
// Combining requires knowing which fields each producer uses
```

**Good** (uniform type, combines with extend):
```rust
fn tick(&self) -> Vec<ServiceEvent>

// Combining is trivial
let mut events = Vec::new();
events.extend(routing.tick());
events.extend(sync.tick());
events.extend(link.tick());
```

**Litmus test:** Can outputs be combined with just `extend()`? If you need field-by-field merging or special logic, the output type is wrong.

### Isolated Mutations

Each piece of mutable state has exactly one writer. This prevents conflicts and makes data flow traceable.

| State | Single Writer | Readers |
|-------|---------------|---------|
| `routing_table` | RoutingService | Core, Extensions (via State) |
| REDS tables | SyncService | Extensions (via State) |
| Session keys | SessionService | Core (encryption) |
| Links map | LinkService | Extensions (via State) |

**Why this matters:**
- No race conditions between services
- Clear ownership: "who mutates this?" has one answer
- Debugging: if state is wrong, you know which service to check

**Anti-pattern:** Multiple services writing to shared state. If two services need to update the same thing, one should own it and the other should request changes via commands/events.

---

## Interface Consistency Standards

### Tick Method Pattern

Components that produce output on each cycle SHOULD follow this pattern:

```rust
// Target pattern for tick-producing components
fn tick(&mut self, ctx: &Context) -> Result<(Stats, Vec<Action>), Error>
```

| Component | Returns | Notes |
|-----------|---------|-------|
| **Engine** (sync) | `TickResult` or `Vec<Action>` | Pure computation, no I/O |
| **Bridge** (async) | `Result<(Stats, Vec<Action>), Error>` | Wraps engine, handles I/O |

### Naming Conventions

| Verb | Semantics | Examples |
|------|-----------|----------|
| `tick` | Main processing cycle, may produce output | `tick()`, `tick_router()` |
| `drain` | Consume and return pending items | `drain_events()`, `drain_ranging()` |
| `get` | Read without side effects | `get_stats()`, `get_neighbors()` |
| `poll` | Check for data without blocking | `poll_events()` |

**Avoid:** `process_*` for operations that secretly call `tick()` (violates local reasoning).

### Self-Reference Pattern

| Pattern | When to Use |
|---------|-------------|
| `&mut self` | Default for tick methods—mutation is expected |
| `&self` with Mutex | Required for async traits in `Arc` (document this!) |

If using `&self` with interior mutability, add this docstring:
```rust
/// Note: Uses `&self` with interior mutability (Mutex) for trait compatibility.
/// This differs from other tick() implementations which use `&mut self`.
```

### Error Handling Pattern

Each boundary has one error type. Follows the Type Naming Convention: `<Boundary>Error`.

- **Outcomes** (valid results) are separate from **Errors** (failures) — see "Outcomes vs Errors" above
- Errors at boundaries **wrap** inner errors via `From` impl
- Errors provide **helper methods** for caller convenience

```rust
// Boundary-specific errors (see Type Naming Convention)
pub enum CoreError { NoRoute(PeerId), PayloadTooLarge, ... }
pub enum ServiceError { Core(CoreError), NoSession(PeerId), ... }
pub enum OrchestratorError { Service(ServiceError), Runtime(RuntimeError), ... }

impl From<CoreError> for ServiceError {
    fn from(e: CoreError) -> Self { Self::Core(e) }
}
```

**Error helper methods:** All errors SHOULD distinguish fatal from recoverable:
```rust
impl ServiceError {
    fn is_fatal(&self) -> bool { ... }      // Should the system stop?
    fn is_transient(&self) -> bool { ... }  // Can we retry?
}
```

### Error Disposition

Rust's `Result` is already the pit of success — `#[must_use]` warns on ignored results, `?` makes propagation trivial. The anti-patterns happen when **return types create error dead ends** where `Result` can't reach.

> **If a function can fail in ways the caller should know about, the return type must carry that information.** A type that forces callers to swallow errors is a design bug — fix the type, not the caller.

Errors must be either handled (retry, fallback, degrade) or propagated (via `?` or return type). If you find yourself writing `debug!(error); continue`, the return type probably doesn't carry failure information — fix the type.

**The log-line test:** If the log line is the *only* thing that happens on error, you're using logging as a substitute for error handling. Logging is observability, not handling.

| Pattern | Handling? |
|---------|-----------|
| `warn!(error); retry()` | Yes — retry is the handling, log is observability |
| `warn!(error); continue` | No — error vanishes after the log |
| `stats.failed += 1` (nobody reads it) | No — phantom counter, fiction observability |

### Observability Requirements

Logging is observability, not error handling (see Error Disposition above). Every boundary handoff should be visible without reading source code.

**Full reference:** `docs/OBSERVABILITY.md` — field schema, pattern guide, querying guide, new-module checklist.

#### Level Conventions

These levels are **not guidelines — they are conventions**. Choosing the wrong level is a bug, same as choosing the wrong error type.

| Level | When | Examples | Compile-Time Behavior |
|-------|------|---------|----------------------|
| `error!` | Failures that affect correctness. System may be in invalid state. | Extension panic, lock poison, invariant violation | Always compiled |
| `warn!` | Degraded operation. System continues but with reduced capability. | Stale sensor, connection failed (retrying), handshake collision | Always compiled |
| `info!` | Lifecycle events. State transitions a human operator cares about. | Init, shutdown, mode change, extension enabled/disabled | Always compiled |
| `debug!` | Per-tick summaries and boundary handoff counts. Diagnostic detail. | "Router tick: 3 items", "Flush: 2 dispatched, 1 deferred" | **Compiled out in release** |
| `trace!` | Per-item detail in hot paths. Individual frame/message processing. | "handle_frame: msg_type=DATA, len=128", "extension tick: 240µs" | **Compiled out in release** |

**The level test:** "If I turn on only `info` in production, can I tell the system is healthy?" If not, something is at the wrong level.

**Compile-time elimination:** `release_max_level_info` in `Cargo.toml` compiles out `trace!` and `debug!` in release builds. Hot-path logging has **zero runtime cost** in production.

#### Span Patterns

Boundary functions use spans for automatic timing and hierarchical context. The pattern depends on the function type:

| Pattern | When | Example |
|---------|------|---------|
| `#[instrument(skip(self), fields(...))]` | Sync `&mut self` methods | `handle_frame`, `SyncEngine::tick` |
| Extract body + `.instrument(span)` | Async methods | Orchestrator tick, bridge ticks |
| Events with `extension = self.name()` | Extension trait objects | Localization, coordination |

**Never** use `span.enter()` guards in async code — they are `!Send`.

Use `field::Empty` + `span.record()` for deferred fields (record outcomes after computation).

Use `follows_from` for causal chains across channel boundaries (trigger → sync).

See `docs/OBSERVABILITY.md` Pattern Guide for full examples.

#### The Silent Stage Rule

> **Every pipeline stage must emit at least one structured event per tick.**

A "pipeline stage" is a function that transforms data from one form to another. If the stage produces no output AND emits no event, the data has vanished — this is a bug in observability, not just a missing log line.

#### Import Convention (Enforced)

Direct imports only. Path-qualified calls are a convention violation. All source files have been converted.

```rust
// GOOD
use tracing::{debug, info, warn};

// BAD — violates convention
tracing::warn!("...");
```

Use field schema names from `docs/OBSERVABILITY.md`. Do not invent new field names.

---

## Boundary Rules

### Engine/Bridge/Service Boundary

**Rule:** External route state queries go through services, not engines or bridges.

```
GOOD: Single query path
NetworkCore.send() → routing_table.is_reachable() → RoutingTable (service-owned)

BAD: Bypassing service
NetworkCore.send() → router.is_reachable() → AodvBridge → AodvEngine
```

The `Router` trait (on `AodvBridge`) does NOT include query methods.
`NetworkCore` holds `Arc<dyn Router>` for I/O only. Route queries go through
`Arc<RwLock<RoutingTable>>`. Calling `router.is_reachable()` is a compile error.

See `docs/ARCHITECTURE_REFORM.md` §I.3 for the full Engine/Bridge/Service pattern.

### Dispatch Boundary

**Rule:** All outgoing traffic flows through Orchestrator.

```
GOOD: Single dispatch point
┌─────────────────────────────────────────────────────────────────┐
│                           ORCHESTRATOR                           │
│  Component.tick() ───┐                                          │
│  Component.tick() ───┼──▶ Unified Dispatch ──▶ Network          │
│  Component.tick() ───┘                                          │
└─────────────────────────────────────────────────────────────────┘

BAD: Fragmented dispatch
┌─────────────────────────────────────────────────────────────────┐
│  Runtime ─────────────────────────────────▶ Network             │
│  Orchestrator ────────────────────────────▶ Network             │
│  NetworkStack ────────────────────────────▶ Network             │
└─────────────────────────────────────────────────────────────────┘
```

**Status:** Unified dispatch implemented via `Orchestrator.TrafficController` (Phase 6).

### L3/L4 Encryption Boundary

| Path | Encryption | Use Case |
|------|------------|----------|
| L3 (broadcast_raw) | **None** | AODV control traffic (HELLO, RREQ, RREP) |
| L4 (send) | **ChaCha20-Poly1305** | Application data, sync records |

**Rule:** Never send sensitive data via L3 raw methods.

### State Boundary

**Rule:** Extensions interact through State, never REDS directly.

```rust
// GOOD: Extension uses State
fn tick(&mut self, state: &dyn State) -> Result<(), ExtensionError> {
    let positions = state.positions();    // Reads snapshot
    state.set_my_position(new_pos);       // Queues write
    state.send(peer, msg_type, payload);  // Queues action
    Ok(())
}

// BAD: Extension accesses REDS
fn tick(&mut self, state: &dyn State) -> Result<(), ExtensionError> {
    let table = self.reds.table::<Position>();  // DON'T DO THIS
}
```

See `docs/STATE_REDS_BOUNDARY.md` and `docs/NETWORK_STATE_BOUNDARY.md` for details.

---

# Part III: Methodology

## Before You Code

### Spec-First Approach

When implementing, your mental model should be:
1. "What does SYSTEM_ARCHITECTURE.md say?"
2. "What does the firmware protocol provide?"
3. "What invariants must hold?"

**NOT:**
1. "How did mesh-core do it?"
2. "Let me copy this pattern and adapt it."

Starting from the spec forces you to understand the *why*. Starting from mesh-core copies the *what* without understanding.

### Write Down Your Understanding First

Before writing code, explicitly document what you understand the spec to require:

```
Understanding from SYSTEM_ARCHITECTURE.md §18.2:
- [ ] Firmware provides link_quality_score directly (don't recalculate)
- [ ] Links start Uninitialized, not with fiction quality values
- [ ] Credit check and decrement must be atomic (single lock)
- [ ] Poll all drivers fairly (rotating offset)
- [ ] Return None when no links available, not link 0
```

After implementation, verify each item against your code. If you can't point to the line that implements each requirement, the implementation is incomplete.

### Identify Boundaries

Before implementing, map which component owns which concept:

| Concept | Owner | NOT Owner |
|---------|-------|-----------|
| State snapshots | Runtime | Extensions |
| Dispatch timing | Orchestrator | NetworkStack |
| Session keys | Security | Messenger |
| Link quality | L2 Coordinator | L3 Router |

---

## During Development

### Don't Rush

The context window is finite. Wasting it on repeated exploration degrades assistance.

**Before ANY code change:**
1. **READ FIRST** - Use Read tool before Edit tool
2. **VERIFY** - Confirm names/locations exist
3. **ANALYZE** - Explain what you found BEFORE changing
4. **ONE CHANGE** - Single logical change at a time
5. **TEST** - Run tests after each change

When user says "don't rush" or "think hard":
1. STOP immediately
2. Acknowledge the signal
3. Read relevant files FIRST
4. Show analysis before next change

### Context Preservation

- **Use planning mode** for complex multi-step tasks
- **Use subagents** to explore without consuming main context
- **Use the Explore agent** for open-ended questions
- **Summarize findings** before implementing

---

## Verification

### The Spec-to-Code Verification Loop

1. **Before coding:** Write down your understanding of what the spec requires
2. **While coding:** For each requirement, write the code AND the test together
3. **After coding:** Re-read the spec and verify each point is implemented
4. **Before committing:** Check against the anti-pattern checklist

### The Three Verification Questions

After every implementation, explicitly ask:

1. **Traceable:** Can I point to the spec section this implements?
2. **Faithful:** Does my code do what the spec says, or something slightly different?
3. **Obvious:** Would someone reading only the spec understand why this code exists?

If the answer to any is "no", stop and fix the disconnect.

### Anti-Pattern Checklist

Before finalizing any implementation, verify you avoided these traps:

| Question | What to Check |
|----------|---------------|
| Implicit state? | Did you use enums, not booleans, for state machines? |
| Recalculating available data? | Does a lower layer already provide this value? |
| Check-then-act? | Are multi-step operations atomic under a single lock? |
| Copying mesh-core? | Did you start from spec, or from "how did mesh-core do it"? |
| Fiction values? | Do uninitialized entities have fake "reasonable" defaults? |
| Superficial tests? | Do tests verify spec invariants, or just that code runs? |
| Interface inconsistency? | Does your tick() match the established pattern? |
| Boundary violation? | Are you dispatching from the right place? |
| Error dead end? | Does the return type give errors somewhere to go? |
| Log-and-swallow? | Is a `debug!`/`warn!` the only thing that happens on error? |
| Phantom counter? | Is a `stats.failed += 1` actually read by anyone? |
| Silent pipeline stage? | Does every pipeline stage emit at least one event per tick? |
| Unwired obligation? | Are all `WiringObligation` return values consumed? |

---

## Testing Philosophy

Tests exist to verify the design is implemented correctly, not just that code runs without crashing.

### Test the Spec, Not the Code

Tests must be derived from design invariants, not from implementation details.

| Design Decision | Test That Verifies It |
|-----------------|----------------------|
| "Firmware quality is used directly" | Assert quality equals injected firmware value, not recalculated |
| "Uninitialized links cannot transmit" | Assert transmission returns `LinkNotReady` error |
| "Credits are atomic" | Inject error after reserve, verify credit is restored |
| "Link selection returns None when exhausted" | Assert `None`, not link 0 |

### The Falsifiability Test

For every test, ask: **"What bug would make this test fail?"**

**Bad test** (passes even if implementation is wrong):
```rust
#[test]
fn link_has_quality() {
    let link = Link::new();
    assert!(link.quality <= 100);  // Trivially true for any u8
}
```

**Good test** (fails if invariant violated):
```rust
#[test]
fn quality_is_firmware_value_not_recalculated() {
    driver.inject_status(quality: 73, ...);
    coordinator.poll_events();
    assert_eq!(coordinator.get_quality(link_id), 73);  // Fails if we recalculate
}
```

If you can't describe a bug that would cause your test to fail, the test is superficial.

### Test Event Flows, Not Internal State

**Bad** (bypasses real behavior, tests nothing):
```rust
links[0].is_up = true;
links[0].quality = 80;
assert!(coordinator.is_link_up(link_id));  // Of course it is—we just set it!
```

**Good** (tests actual event processing):
```rust
driver.inject_event(Status { quality: 80, radio_state: Ready, ... });
coordinator.poll_events();
assert!(coordinator.is_link_up(link_id));  // Verifies Status processing works
```

### Layered Testing

| Question | Layer | Tool |
|----------|-------|------|
| Does my algorithm work? | Unit | MockState, MockPlatform |
| Do extensions interact across nodes? | Extension | MultiNodeTestbed |
| Do layers interact correctly? | Integration | MockTransport |
| Does the fleet form? | Fleet | tropo-sim scenarios |

**Principle:** Use the simplest layer that answers your question.

```rust
// Multi-node extension test with MultiNodeTestbed
let mut testbed = MultiNodeTestbed::with_topology(MockTopology::line(&[1, 2, 3]));
testbed.add_extension(PeerId(1), LocalizationExtension::new());
testbed.step().unwrap();
assert!(testbed.visible_peers(PeerId(1)).contains(&PeerId(2)));
```

### Trait-Based Testing

Production traits ARE the testing interface:

```rust
// Unit test with MockState
let state = MockState::new();
state.inject_position(peer_id, test_pos);
extension.tick(&state)?;
assert_eq!(state.my_position(), expected);
```

No special harness needed—mock implementations for unit tests, real implementations for integration.

### Tests Encode Contracts

| Weak Test | Strong Test |
|-----------|-------------|
| `assert!(result.is_ok())` | `assert_eq!(result.unwrap().x, 10.0)` |
| "No panic" | "Returns expected Position" |
| "Compiles" | "Round-trip serialization preserves data" |

### Test Infrastructure Is Production Code

MockState, MockPlatform, MockNetworkHandle:
- Same quality standards as production code
- Need their own unit tests
- Document like production APIs

### Boundary Integration Tests

Unit tests with mocks verify each side of a boundary independently. Boundary tests verify the **wiring** — that data actually flows across the boundary when both sides use real implementations.

#### When to Write a Boundary Test

| Signal | Example |
|--------|---------|
| Two modules share a channel | TriggerDispatcher → SyncEngine |
| One module's output is another's input | AodvEngine frames → handle_frame dispatch |
| Data crosses an Arc<dyn Trait> boundary | Platform → Runtime (SensorReading) |
| A shared resource (Arc<RwLock<T>>) bridges modules | RoutingTable between AodvBridge and NetworkCore |

#### Design Rules

1. **Cross exactly one boundary.** Test A→B, not A→B→C→D. If the test fails, you know which boundary is broken.
2. **Real implementations on both sides.** No mocks at the boundary being tested. Mocks are allowed for *other* dependencies that the test doesn't care about.
3. **Falsifiable.** State what bug would make the test fail. If you can't, the test is superficial (see "The Falsifiability Test" above).
4. **Named by boundary.** File name: `tests/boundary_<source>_<sink>.rs`. This makes the test discoverable from the boundary it covers.

#### Contract

Each boundary test MUST include a comment at the top:

```rust
//! Boundary test: <source module> → <sink module>
//!
//! Verifies: <what data flows across>
//! Falsifiable: <what bug would make this fail>
//! Bug class: <what category of bug this catches>
```

#### Relationship to Unit Tests

Boundary tests do NOT replace unit tests. They complement them:

| Test Type | What It Catches | Example |
|-----------|----------------|---------|
| Unit test | Algorithm bug in AodvEngine | Wrong metric formula |
| Boundary test | Wiring bug between modules | MSG_TYPE_DATA prefix missing |
| Integration test | Multi-boundary flow | End-to-end message delivery |

---

# Part IV: Domain Knowledge

## Critical Constraints

These are non-negotiable. Violating them creates systemic problems.

| Constraint | Rationale |
|------------|-----------|
| **SE calls are COLD PATH ONLY** | Per-packet crypto uses pre-derived session keys. Never call Secure Element in the routing hot path. |
| **Extensions are SYNC, not async** | `tick()` runs to completion. No await, no blocking I/O. Read state → Compute → Write state → Return. |
| **Single-writer rule for REDS** | Each node writes only its own entries. Conflicts are impossible by design. |
| **Gateway runs IN tropo-core** | No IPC overhead for internal access. Apps connect via ZeroMQ from outside. |

---

## Anti-Patterns from mesh-core

**This is a clean-slate architectural reform, not a port.** These patterns caused bugs in mesh-core. Do not replicate them.

### Patterns to Avoid

| Anti-Pattern | What mesh-core Does | Why It's Wrong |
|--------------|---------------------|----------------|
| Implicit link state | `is_link_up()` returns true if ID is valid | Doesn't check actual health or timeout |
| Recalculating available data | L2 computes quality from PER | Firmware already provides EMA-smoothed score |
| No timeout detection | Links stay "up" forever | Crashed firmware never detected |
| Fiction initial values | New links have quality=50 | Hides uninitialized state, causes silent misbehavior |
| Parallel arrays | Separate arrays for drivers, links, trackers | Fragile index correlation, easy to desync |
| Check-then-act | Read credits in one lock, decrement in another | TOCTOU race under concurrency |

### Architectural Patterns to Avoid

- The deeply nested generic composition (`Messenger<Router<TRXCoordinator<...>>>`)
- The MeshNode trait design (it conflates internal and external needs)
- The TestHarness approach (too coupled, replaced by trait mocks)
- The L5 service model (replaced by sync Extensions)
- The async-everywhere pattern (Extensions are explicitly sync)

### What to Reference from mesh-core

Only these specific items, and verify they still apply:

| Reference | Verify Against |
|-----------|----------------|
| Protocol byte layouts (USB CDC frames, wire formats) | protocol.yaml |
| Algorithm implementations (AODV math, trilateration) | SYSTEM_ARCHITECTURE.md |
| Crypto primitives (ChaCha20-Poly1305 usage) | SECURITY_ARCHITECTURE.md |

### Justification Required

If you must use any mesh-core pattern, document WHY in a code comment:

```rust
// NOTE: Using mesh-core's AODV metric formula (1000 - quality*10 + hops*100)
// Verified against SYSTEM_ARCHITECTURE.md §18.3.2
```

---

## API Completeness Rules

### Symmetric Operations Rule

Every state-changing operation MUST have its inverse:

| Operation | Inverse |
|-----------|---------|
| `join_group()` | `leave_group()` |
| `subscribe()` | (drop Subscription) |
| `start()` | `stop()` |

### CRUD Completeness Rule

Resource-managing APIs need full lifecycle:

| Pattern | Example |
|---------|---------|
| Create | `add_route()` |
| Read | `get_route()` |
| Delete | `remove_route()` |
| List | `list_routes()` |
| Bulk delete | `clear_routes()` |

---

## Terminology

Precise definitions to prevent confusion:

| Term | Meaning |
|------|---------|
| **State** | Extension's sync interface to distributed data (RealState/MockState) |
| **REDS** | Replicated Embedded Data Store - SQLite-backed CRDTs |
| **Extension** | Sync, deterministic module that runs in tick loop |
| **Refresh** | Pull latest REDS data into State snapshots (start of tick) |
| **Flush** | Commit queued writes to REDS, return actions (end of tick) |
| **Action** | Queued network operation (send/broadcast) executed by Runtime |
| **NetworkHandle** | Async interface to L4 Messenger (Runtime only, not extensions) |
| **Platform** | Hardware abstraction (sensors, actuators) |
| **Tick** | Single extension execution cycle: refresh → tick → flush |
| **Engine** | Sync computation component (e.g., AodvEngine, SyncEngine) |
| **Bridge** | Async wrapper that connects Engine to I/O (e.g., AodvBridge, SyncNetworkBridge) |

### Correct Usage

- "Extension reads from **State**, not REDS"
- "**Refresh** populates snapshots, **flush** commits writes"
- "**Actions** are queued, not executed immediately"
- "**Engine** produces actions, **Bridge** collects and returns them"

---

# Part V: Reference

## Code Quality

Run before every commit:

```bash
cargo fmt --check
cargo clippy --all-targets -- -D warnings
cargo test --lib
```

Clippy warnings are errors. Fix them.

---

## Git Discipline

**Main is sacred.** Never push directly to main.

```bash
git checkout -b feature/my-work
# ... work, commit ...
git push -u origin feature/my-work
# Create PR -> review -> merge
```

**Commit format:**
```
<type>: <subject>

<body (optional)>

claude
```

**Footer is EXACTLY:** `claude` (one word, nothing else)

**Types:** `feat`, `fix`, `docs`, `refactor`, `test`, `chore`

---

## Related Repositories

| Repository | Purpose |
|------------|---------|
| **tropo-sentinel** | Lifecycle manager (parent process) - does not exist yet |
| **tropo-sim** | Fleet simulation (what tests tropo-core) |
| **mesh-core** | Legacy router (patterns to learn from, not copy) |
| **mesh-docs-restricted** | Architecture documentation |

---

## Open Items

These require specification before implementation:

| Item | Status |
|------|--------|
| Gateway topic schema | Needs Protobuf definitions |
| L1 driver abstraction | Needs trait design |
| Configuration format | TOML structure TBD |

**Resolved:**
- Extension trait signature (Section 20.2)
- State interface API (Section 20.3)
- Error handling: Unified errors at boundaries, internal errors per-layer
- Interface unification: `TickOutput<S, A>` pattern in `src/tick.rs`
- Dispatch unification: `Orchestrator.TrafficController` (Phase 6)

**Do not implement against undefined interfaces.** Wait for specification or propose one.

---

**Last Updated:** 2026-01-27
**Conventions:** See `../mesh-docs-restricted/convention/STANDARDS.md`
