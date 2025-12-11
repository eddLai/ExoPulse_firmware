# TCP Demo Data Flow Debug Log

## Date: 2024-11-21

## Issue Summary
TCP connection established successfully, but **no data is being transmitted** from experiment_menu.py to the client.

## Observed Behavior

### Client Side (test_exo_tcp_client.py)
```
Connecting to 127.0.0.1:9998...
✓ Connected to 127.0.0.1:9998
------------------------------------------------------------
Waiting for data... (Ctrl+C to stop)
------------------------------------------------------------
⚠ Timeout - no data received for 2 seconds
⚠ Timeout - no data received for 2 seconds
⚠ Timeout - no data received for 2 seconds
... (repeated)
```

### Server Side (experiment_menu.py)
- UI shows realtime data: `Receiving data | 1357 points | t=15.82s`
- Charts displaying: `hip_exo_r_rotation`, `hip_exo_l_rotation`, `hamstrings_r`, `hamstrings_l`
- Terminal shows: `[Exo] Angle L: xxx° R: xxx° | Torque L: xxxNm R: xxxNm` (every 3 seconds)
- `[ExoDataBroadcaster] Started on port 9998` confirmed

## Analysis

### What Works
1. ✅ ExoDataBroadcaster TCP server starts on port 9998
2. ✅ Client can connect to the server
3. ✅ Realtime data is being generated (~100Hz)
4. ✅ `_on_exo_angles_received()` callback is being called (terminal output confirms this)

### What Doesn't Work
1. ❌ Data is NOT being sent to connected TCP clients
2. ❌ No `[ExoDataBroadcaster] Client connected: ...` message in terminal

## Suspected Root Cause

The `broadcast()` method in `exo_data_broadcaster.py` checks:
```python
if not self.running or not self.clients:
    return  # Silently skip if no clients
```

**Possible issues:**
1. Client connection not being registered in `self.clients` list
2. `_accept_clients()` thread not running properly
3. `exo_broadcaster` attribute not accessible in callback context

## Files Involved
- `/home/rogeric/ExoPulse/standalone_moe_config_generator/components/exo_data_broadcaster.py`
- `/home/rogeric/ExoPulse/standalone_moe_config_generator/experiment_menu.py`
- `/home/rogeric/ExoPulse/depRL/deploy_IoT/ExoPulse_firmware/test_exo_tcp_client.py`

## Next Steps
1. Add debug logging to `_accept_clients()` to verify client registration
2. Add debug logging to `broadcast()` to verify it's being called with clients
3. Check if `self.exo_broadcaster.client_count` is > 0 after connection
