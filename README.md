![GitHub Actions Workflow Status](https://img.shields.io/github/actions/workflow/status/vp-supplementary/nucleo-f302-spwm/ci.yml)

# Nucleo F302R8 SPWM example

This example shows how to use the [spwm](https://github.com/vpetrigo/spwm) crate to have a software controlled PWM
output with the given frequency.

That example configures That example configures 4 channels:

- PC9: 10 Hz, 50% duty cycle
- PC8: 50 Hz, 10% duty cycle
- PC6: 500 Hz, 50% duty cycle
- PC5: 250 Hz, 64% duty cycle

Oscillogram that shows PC9 and PC8 output waveforms:

![pwm_oscillogram](docs/oscillogram.png)4 channels:

- PC9: 10 Hz, 50% duty cycle
- PC8: 50 Hz, 10% duty cycle
- PC6: 500 Hz, 50% duty cycle
- PC5: 250 Hz, 64% duty cycle

Oscillogram that shows PC9 and PC8 output waveforms:

![pwm_oscillogram](docs/oscillogram.png)

## License

<sup>
Licensed under either of <a href="LICENSE-APACHE">Apache License, Version 2.0</a> or
<a href="LICENSE-MIT">MIT license</a> at your option.
</sup>

<br>

<sub>
Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in this codebase by you, as defined in the Apache-2.0 license,
shall be dual licensed as above, without any additional terms or conditions.
</sub>