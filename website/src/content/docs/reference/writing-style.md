---
title: Writing style
description: How the pages on this site are written. Based on ASD-STE100 Simplified Technical English.
---

The pages on this site follow
[ASD-STE100 Simplified Technical English](https://www.asd-ste100.org/) where it
helps. The aim is text that is clear to every reader, including readers whose
first language is not English. The rules are a strong preference, not a strict
requirement.

## Rules

1. **One topic per sentence, one instruction per step.**
2. **Short sentences.** Procedures: 20 words or fewer. Descriptions: 25 words or fewer.
3. **Short paragraphs.** One topic, six sentences or fewer.
4. **Active voice.** "The ATOM sends the packet", not "The packet is sent".
5. **Imperative in procedures.** "Connect the cable." Put a condition before the instruction: "If the LED is red, press the button."
6. **One word for one thing.** Use the terms below. Do not change the term for variety.
7. **Simple verb tenses:** present, simple past, future. No "-ing" forms as verbs.
8. **Use articles** ("the", "a") where they are normal in English.
9. **Numbers with units**, and the measured value, not "fast" or "slow".
10. **Warnings before the step** they apply to. Use the `caution` and `danger` boxes.

Technical names (register names, function names, file names) are allowed as they are.

## Terms

| Use | Do not use |
| --- | --- |
| ATOM | M5, controller, end board |
| servo bus | serial bus, servo line |
| laptop | host, PC, computer |
| FT232R | FTDI, USB adapter (except to explain) |
| base | transponder, bottom board |
| plan | trajectory file, path |
| recording | log, capture (for player output) |
| goal position, goal speed | target, setpoint (for servo registers) |
| hold the pose | lock, freeze |
| lag compensation | time shift, feedforward (for this method) |

## Keep the site true

- This site is the source of truth. When code changes, change the page in the same commit.
- Write what is measured, with the date and the numbers. Mark what is not verified.
- Do not delete a wrong statement silently. Correct it, and say what changed if readers can have seen it.
