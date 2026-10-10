# Xeno Crisis MSU-1 packs (sd2snes)

The music comes from an MSU-1 pack next to the ROM; the sound effects still come from the game. This firmware does not decode the game's own Opus streams, so a pack is the only source of music.

## Files

Standard MSU-1 layout, named after the ROM file (here `Xeno Crisis.sfc`):

| File | Content |
|---|---|
| `Xeno Crisis.msu` | Must exist. Its content is not used (the game reads no MSU-1 data); an empty file is fine. |
| `Xeno Crisis-<n>.pcm` | Track n: `MSU1`, a 32-bit little-endian loop point in samples, then 44.1 kHz 16-bit signed little-endian stereo. |

Without the `.msu` the game runs with its sound effects only, on every board.

## Track numbers

Track n is the game's music stream n. The length is the game's own version, for reference; a pack's tracks may be longer or shorter.

| n | Game track | Game length | Notes |
|---|---|---|---|
| 1 | MUSIC_0_INTRO_VOICED | 0:31.6 | Played once (boot intro). Soundtrack: "Intro". |
| 2 | MUSIC_10_ALARM | 0:05.4 | Played once or looped, depending on the situation. Soundtrack: "Sound Effects part 1", 3.08-8.52 s (cut to the game's 5.44 s). |
| 3 | MUSIC_1_TITLE | 0:36.4 | Loops. Soundtrack: "Title". |
| 4 | MUSIC_2_INGAME_0_INTRO | 0:02.8 | Intro of 5. Soundtrack: "Perimeter (Area 1)" (its intro). |
| 5 | MUSIC_2_INGAME_0_LOOP | 1:47.0 | Loops. Soundtrack: "Perimeter (Area 1)" (its loop). |
| 6 | MUSIC_2_INGAME_1 | 2:39.8 | Loops. Soundtrack: "Facility (Area 2)". |
| 7 | MUSIC_2_INGAME_2 | 2:08.6 | Soundtrack: "Dunes (Area 3)". |
| 8 | MUSIC_2_INGAME_3_INTRO | 0:10.6 | Intro of 9. Soundtrack: "Nest (Area 4)" (its intro). |
| 9 | MUSIC_2_INGAME_3_LOOP | 1:57.4 | Soundtrack: "Nest (Area 4)" (its loop). |
| 10 | MUSIC_2_INGAME_4_INTRO | 0:22.8 | Intro of 11. Soundtrack: "Forest (Area 5)" (its intro). |
| 11 | MUSIC_2_INGAME_4_LOOP | 1:21.4 | Soundtrack: "Forest (Area 5)" (its loop). |
| 12 | MUSIC_2_INGAME_5_INTRO | 0:08.2 | Intro of 13. Soundtrack: "Lab (Area 6)" (its intro). |
| 13 | MUSIC_2_INGAME_5_LOOP | 1:56.0 | Soundtrack: "Lab (Area 6)" (its loop). |
| 14 | MUSIC_2_INGAME_6_INTRO | 0:32.0 | Intro of 15. Soundtrack: "HQ (Area 7)" (its intro). |
| 15 | MUSIC_2_INGAME_6_LOOP | 1:48.6 | Soundtrack: "HQ (Area 7)" (its loop). |
| 16 | MUSIC_3_BOSS_0 | 0:57.4 | Soundtrack: "Boss 1". |
| 17 | MUSIC_3_BOSS_1 | 0:35.8 | Soundtrack: "Boss 2". |
| 18 | MUSIC_3_BOSS_2_INTRO | 0:00.8 | Intro of 19. Soundtrack: "Boss 3" (its intro). |
| 19 | MUSIC_3_BOSS_2_LOOP | 0:40.8 | Soundtrack: "Boss 3" (its loop). |
| 20 | MUSIC_3_BOSS_3 | 0:45.2 | Soundtrack: "Boss 4". |
| 21 | MUSIC_4_CUTSCENE | 0:44.2 | Loops. Soundtrack: "Continue" (its first 44.1 s are this stream; the soundtrack piece goes on for 2:25). |
| 22 | MUSIC_5_SHOP | 0:22.4 | Loops. Soundtrack: "Stage Clear". |
| 23 | MUSIC_6_CONTINUE | 0:34.8 | Loops. Soundtrack: "Upgrade" (the game's stream is one 30.3 s pass plus a 4.4 s ring-out; the soundtrack loops the pass). |
| 24 | MUSIC_7_STAGE_CLEAR | 0:07.2 | Played once. Soundtrack: "Game Over" (the audio matches this stream, not 25). |
| 25 | MUSIC_8_GAME_OVER | 0:12.0 | Played once. Soundtrack: "Cutscene". |
| 26 | MUSIC_9_ENDING | 2:00.0 | Loops (from the start, after its fade). Soundtrack: "Ending". |

Some of the firmware's stream names do not match the soundtrack titles ("Game Over" is stream 24, "Continue" 21, "Stage Clear" 22, "Upgrade" 23, "Cutscene" 25): the slot is the stream whose music it is, found by comparing the audio.

## How tracks are played

- **Looping:** the game decides whether a track loops (the Notes column says which, from the firmware's own calls). A looping track plays with MSU-1 repeat, so set the `.pcm` loop point where the loop should restart (0 = the whole track, which is what the game itself does). A track the game plays once (the voiced intro, jingles) ignores the loop point.
- **Intro + loop pairs** (4/5, 8/9, 10/11, 12/13, 14/15, 18/19): the intro plays once, and the loop part starts as soon as the intro `.pcm` ends. The intro's length does not have to match the game's. If the soundtrack has the two parts as one piece, either split it at the same point, or put the whole piece in the loop track with a loop point after the intro and an empty or very short intro track.
- **Missing tracks** are silent; the game carries on.
- **Pausing** the game pauses the MSU-1 track, and unpausing continues it.
- **Level:** the music plays at full MSU-1 volume. Normalize the pack the way MSU-1 packs usually are, around -20 LUFS integrated (the soundtrack's masters are about -12 LUFS, much louder); the sd2snes menu's MSU-1 volume boost setting applies.

## Converting a soundtrack file

Example for track 1 (the soundtrack's "Intro" is the game's MUSIC_0_INTRO_VOICED): measure the loudness, set the gain to reach -20 LUFS, resample to 44.1 kHz, and add the 8-byte header (loop point 0 here):

```sh
ffmpeg -i "01 - Intro.flac" -af ebur128=framelog=quiet -f null -          # "I: -11.8 LUFS" -> gain -8.2 dB
ffmpeg -i "01 - Intro.flac" -af "volume=-8.2dB,aresample=44100:resampler=soxr:dither_method=triangular" \
       -ac 2 -f s16le body.raw
python3 -c "import struct,sys; sys.stdout.buffer.write(b'MSU1'+struct.pack('<I',0)+open('body.raw','rb').read())" \
       > "Xeno Crisis-1.pcm"
```

For a looping track, put the loop point (in 44.1 kHz samples from the start of the audio data) in place of the `0`. msupcm++ does all of this from a JSON track list, if you prefer a tool.

Saving works as usual (`.srm`). While a pack is used, the firmware checks the save RAM once a second during play and writes the `.srm` when it changed.
