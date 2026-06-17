# BrainBoard 15 Datasheet Discovery

This file is the working source of truth for unanswered questions, partial answers, and confirmed facts for the BB15 hardware datasheet.

Status:
- Product: BrainBoard 15 / BB15
- Document target: hardware datasheet for integrators
- Compatibility positioning: Nicla form-factor compatible
- Current source quality: software repo facts + manual answers

## Working Rules
- Put answers directly after each question on the `Answer:` line.
- If the answer is not known yet, leave `Answer:`
- If the answer is partial, write the partial answer and append `(partial)`.
- If the answer is confirmed, append `(confirmed)`.
- Do not delete old answers; update them in place.
- If a previous answer was wrong, replace it and append `Replaces previous answer`.
- If useful, add `Source:` and `Date:` lines under the answer.

## To-Do List
- [ ] Confirm official product naming, SKU, and board revision scheme.
- [ ] Confirm exact compatibility statement for Nicla form factor and validated hosts.
- [ ] Collect full exposed pin/pad list.
- [ ] Collect power rails, logic domains, and absolute/recommended ratings.
- [ ] Confirm flash part, capacity, voltage, and runtime usage.
- [ ] Confirm level shifter behavior and affected signals.
- [ ] Collect mechanical dimensions, thickness, heights, and orientation.
- [ ] Confirm reset, boot, and bring-up requirements.
- [ ] Confirm any compliance/manufacturing facts safe to publish.
- [ ] Mark all remaining gaps that block a production-ready datasheet.

## Known Facts Already Derived From the Repo
- Demo host is Nicla Vision.
- Demo-visible wiring:
  - `D2 / PA_10` proceed-enable
  - `D3` `AKD1500 RESET_N`
  - `D7` AKD chip select
  - `D1` bridge / flash chip select
  - SPI on `D8`, `D9`, `D10`
  - I2C expander `0x43` on `D11`, `D12`
- Software boot flow:
  - drive proceed high
  - configure expander
  - assert AKD reset
  - force boot strap through expander `P0` low
  - release reset
  - continue with flash/model operations
- Product concept already stated by user:
  - module for AKD1500
  - includes flash
  - includes level shifters
  - Nicla form factor

Rule:
- Do not duplicate these as open questions unless the repo fact is suspected to be incomplete or incorrect.

## A. Product Definition
- What is the official one-line product description?
  Answer:
  Source:
  Date:
- What exact market/application categories should the datasheet state?
  Answer:
  Source:
  Date:
- What is the official SKU or ordering code?
  Answer:
  Source:
  Date:
- What hardware revisions exist, and which revision does this datasheet cover?
  Answer:
  Source:
  Date:
- Is BB15 a single fixed product or a family with variants?
  Answer:
  Source:
  Date:

## B. Compatibility and Positioning
- What exact Nicla compatibility claim is safe to make?
  Answer:
  Source:
  Date:
- Which host boards are officially validated?
  Answer:
  Source:
  Date:
- Is the module intended only for Nicla Vision, or for a wider Nicla-compatible ecosystem?
  Answer:
  Source:
  Date:
- Are there known host-side caveats or unsupported pairings that must be stated?
  Answer:
  Source:
  Date:

## C. Functional Overview
- What are the major onboard functional blocks besides AKD1500?
  Answer:
  Source:
  Date:
- What exact role does the onboard flash serve?
  Answer:
  Source:
  Date:
- What exact role do the level shifters serve?
  Answer:
  Source:
  Date:
- Are there any onboard regulators, clock sources, reset supervisors, or expanders that should be called out explicitly?
  Answer:
  Source:
  Date:
- Which signals are exposed to the host and which are internal-only?
  Answer:
  Source:
  Date:

## D. Power and Electrical
- What are the allowed input supply rails and where are they applied?
  Answer:
  Source:
  Date:
- What is the nominal operating voltage of the module logic?
  Answer:
  Source:
  Date:
- What onboard rails are generated locally?
  Answer:
  Source:
  Date:
- What are the absolute maximum ratings?
  Answer:
  Source:
  Date:
- What are the recommended operating conditions?
  Answer:
  Source:
  Date:
- What logic levels appear on exposed digital interfaces?
  Answer:
  Source:
  Date:
- Are any pins 5 V tolerant?
  Answer:
  Source:
  Date:
- Are there required sequencing rules for power, reset, or boot?
  Answer:
  Source:
  Date:
- Are there carrier design requirements such as bulk capacitance, decoupling, or current capability?
  Answer:
  Source:
  Date:
- Are there any measured current numbers safe to publish for idle, boot, or active operation?
  Answer:
  Source:
  Date:

## E. Pinout and Interfaces
- Provide the full exposed pin or pad list.
  Answer:
  Source:
  Date:
- For each exposed pin/pad, what is the name, direction, and function?
  Answer:
  Source:
  Date:
- For each exposed pin/pad, what is the voltage domain?
  Answer:
  Source:
  Date:
- For each exposed pin/pad, what is the default or reset state?
  Answer:
  Source:
  Date:
- For each exposed pin/pad, are there pull-ups, pull-downs, or keepers?
  Answer:
  Source:
  Date:
- Which pins are required for minimum operation?
  Answer:
  Source:
  Date:
- Which pins are optional, reserved, test-only, or no-connect?
  Answer:
  Source:
  Date:
- Which pins are shared, multiplexed, or mode-dependent?
  Answer:
  Source:
  Date:
- Should the datasheet show Nicla-facing names, module-native names, or both?
  Answer:
  Source:
  Date:

## F. AKD1500 Boot and Runtime Behavior
- What boot modes are supported by the hardware?
  Answer:
  Source:
  Date:
- Which boot mode is the intended shipping mode?
  Answer:
  Source:
  Date:
- Which pins or controls determine boot mode?
  Answer:
  Source:
  Date:
- What reset timing requirements should be published?
  Answer:
  Source:
  Date:
- Is external-flash execution the only supported path or just the validated default?
  Answer:
  Source:
  Date:
- What minimum host bring-up sequence should integrators follow?
  Answer:
  Source:
  Date:
- Are there failure modes or recovery steps worth documenting?
  Answer:
  Source:
  Date:

## G. Flash and Storage
- What exact flash part number is populated?
  Answer:
  Source:
  Date:
- What is the flash capacity?
  Answer:
  Source:
  Date:
- What is the flash operating voltage?
  Answer:
  Source:
  Date:
- What bus/interface mode is used for the flash?
  Answer:
  Source:
  Date:
- Is the flash dedicated to model/program storage, firmware storage, or both?
  Answer:
  Source:
  Date:
- Are there published address map or reserved-region constraints?
  Answer:
  Source:
  Date:
- Are alternate flash parts approved?
  Answer:
  Source:
  Date:

## H. Level Shifters and Signal Translation
- Which exact level shifter parts are used?
  Answer:
  Source:
  Date:
- Which signals pass through level shifting?
  Answer:
  Source:
  Date:
- What directionality applies to those translated signals?
  Answer:
  Source:
  Date:
- What voltage domains exist on each side of the level shifters?
  Answer:
  Source:
  Date:
- Are there speed, loading, or protocol limitations the datasheet should note?
  Answer:
  Source:
  Date:

## I. Mechanical
- What are the exact board dimensions?
  Answer:
  Source:
  Date:
- What is the board thickness?
  Answer:
  Source:
  Date:
- What is the weight?
  Answer:
  Source:
  Date:
- What is the maximum top-side component height?
  Answer:
  Source:
  Date:
- What is the maximum bottom-side component height?
  Answer:
  Source:
  Date:
- What orientation convention should be used for front/back views?
  Answer:
  Source:
  Date:
- Are there keepout or placement restrictions?
  Answer:
  Source:
  Date:
- Is there a recommended land pattern or carrier-footprint note that should appear?
  Answer:
  Source:
  Date:

## J. Environmental and Reliability
- What is the operating temperature range?
  Answer:
  Source:
  Date:
- What is the storage temperature range?
  Answer:
  Source:
  Date:
- Are there humidity, handling, or ESD limits to publish?
  Answer:
  Source:
  Date:
- Are there duty-cycle or thermal constraints that affect safe operation?
  Answer:
  Source:
  Date:

## K. Manufacturing and Compliance
- Which compliance claims are already confirmed?
  Answer:
  Source:
  Date:
- Is RoHS status known?
  Answer:
  Source:
  Date:
- Is REACH status known?
  Answer:
  Source:
  Date:
- Are there assembly or reflow handling requirements for integrators?
  Answer:
  Source:
  Date:
- Are there any statements that must be omitted until formal certification is complete?
  Answer:
  Source:
  Date:

## L. Documentation Assets Still Needed
- Do we have a trusted pinout figure?
  Answer:
  Source:
  Date:
- Do we have a trusted block diagram?
  Answer:
  Source:
  Date:
- Do we have a trusted mechanical drawing?
  Answer:
  Source:
  Date:
- Do we have front and back board images suitable for the datasheet?
  Answer:
  Source:
  Date:
- Do we have a revision-history policy for the document?
  Answer:
  Source:
  Date:

## Current Blockers for Drafting the Datasheet
- Official product naming, SKU, and revision coverage:
- Safe Nicla compatibility statement and validated hosts:
- Full pin/pad inventory with direction, domain, and reset state:
- Electrical limits and operating conditions:
- Flash part, capacity, voltage, and usage model:
- Level shifter parts, domains, and affected signals:
- Mechanical dimensions, heights, and orientation convention:
- Compliance/manufacturing statements safe to publish:

## Appendix A: Instructions for Future LLM Updates
- Treat this file as the live source of truth for incremental datasheet discovery.
- Read the whole file before making any suggestion or draft.
- Preserve all existing answers unless explicitly contradicted by newer user input.
- Distinguish between:
  - repo-derived facts
  - user-supplied confirmed facts
  - partial answers
  - unknowns
- Never silently invent numeric specs, electrical limits, part numbers, or compatibility claims.
- When converting answers into datasheet prose, only use facts marked confirmed, or clearly label partial values as provisional.
- Add follow-up questions only where the answer is still needed for a production-ready datasheet.
- Prefer editing unanswered or partial sections instead of creating duplicate questions elsewhere.
- When a user gives a free-form answer in chat, place it under the most relevant existing question instead of opening a new section.
- If a new answer invalidates an old one, replace the old answer and note that it supersedes the earlier statement.
- Keep the file human-readable first and machine-usable second.
- Do not reorder major sections unless the user asks.
- Maintain the `Answer:` line directly under each question so the user can continue answering inline over several days.

Workflow:
- Step 1: read this file
- Step 2: identify unanswered and partial questions
- Step 3: ask only the minimum next questions needed
- Step 4: update this file in place
- Step 5: draft datasheet sections only from confirmed information
