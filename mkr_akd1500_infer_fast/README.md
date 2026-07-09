# MKR1000 AKD1500 Runtime

This sketch is intended for evaluation on `arduino:samd:mkr1000` with the
current AKD1500 library state in this repo.

Important compatibility note:

- The committed runtime/library contract in this repo is still `Akida 2.5.0`.
- The bundled metadata in [model_metadata.cpp](/home/cto/Desktop/BrainBoard15_arduino_library/mkr_akd1500_infer_fast/model_metadata.cpp:1) matches that contract.
- The sketch does not carry the full model payload. It carries only the
  size-prefixed `ProgramInfo` blob in `program_info_blob.cpp`.
- The full matching model must already be staged in BB15 external flash at
  offset `0`.

## Compile

```bash
arduino-cli compile \
  --fqbn arduino:samd:mkr1000 \
  --library /home/cto/Desktop/BrainBoard15_arduino_library \
  /home/cto/Desktop/BrainBoard15_arduino_library/mkr_akd1500_infer_fast
```

## Expected runtime behavior

- Serial baud: `115200`
- Startup prints:
  - `START`
  - `MODE:PREFLASHED`
  - `B:OK`
  - `PI:640`
  - `L:OK`
- Inference loop prints lines like:
  - `#0 ST:0 ID:1 MS:42`

## What this sketch assumes

- MKR1000 is wired to the BB15/AKD1500 hardware per the local helper headers.
- The external flash already contains the model data that matches the committed
  `ProgramInfo` blob.
- This is an evaluation sketch, not a model-flashing sketch.
