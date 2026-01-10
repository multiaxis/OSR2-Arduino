# tcode

Reference implementations of TCode, derived from work by [Tempest](https://www.patreon.com/tempestvr).

For documentation on the tcode protocol, see [tcode-spec](https://github.com/multiaxis/tcode-spec).

## [tcode-osr2.ino](tcode-osr2.ino)

Drives OSR2-class devices.

| Release Date | Version | Notes                                                                                   |
| -----------: | :-----: | :-------------------------------------------------------------------------------------- |
|   2023-11-09 |  v3.6   | Buttons changed from Prev/Play/Next to left/ok/right                                    |
|   2023-11-04 |  v3.5   | TCode library moved into a separate file. Buttons functionality added                   |
|   2022-03-05 |  v3.4   | Support for T-wist 4 added - standard servo now default, parallax is an option          |
|   2021-11-03 |  v3.3   | Low speed vibration channel start function added                                        |
|   2021-08-24 |  v3.2   | Range preference variable storage bug fix                                               |
|   2021-07-07 |  v3.1   | Buffer overload bug fix by limiting to 3x4 T-Code channels, axis auto-smoothing for live commands added |
|   2021-05-08 |  v3.0   | TCode v0.3 compatible                                                                   |
