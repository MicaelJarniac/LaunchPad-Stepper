[![GitHub license](https://img.shields.io/github/license/MicaelJarniac/LaunchPad-Stepper?style=flat-square)](https://github.com/MicaelJarniac/LaunchPad-Stepper/blob/master/LICENSE)
[![GitHub issues](https://img.shields.io/github/issues/MicaelJarniac/LaunchPad-Stepper?style=flat-square)](https://github.com/MicaelJarniac/LaunchPad-Stepper/issues)
![GitHub repo size](https://img.shields.io/github/repo-size/MicaelJarniac/LaunchPad-Stepper?style=flat-square)

# LaunchPad Stepper
A [MSP-EXP430G2][launchpad] _LaunchPad_ firmware to control the [BOOST-DRV8711][stepperdriver] Stepper Motor _BoosterPack_ through a simplified UART protocol.

## Installation
The included files must be imported to a [Code Composer Studio][ccs] project correctly created for the target board.

1. [Download][ccsdown] and install _Code Composer Studio_ (CCS) 9+
1. Download this repository and extract if needed
1. Launch CCS and create a new project
	- **File** > **New** > **CCS Project**
		- **Target:** **MSP430G2553** (select the board you're using)
		- **Project templates and examples:** **Empty Projects** > **Empty Project**
1. Ensure that the new project is selected in the **Project Explorer**
1. Import this repository's contents to the new project
	- **File** > **Import...**
		- **General** > **File System**
			- **From directory** > **Browse...**
			- Find and select the downloaded repository folder
			- **Select All**
			- Check **Overwrite existing resources without warning**
			- Uncheck **Create top-level folder**
			- **Finish**
1. **Project** > **Build Project**
1. **Flash**

## Usage
Read the [message template][template] for an overview of how the messages are composed.

Check out [StepperComms][steppercomms], a Python script for building the messages.

## Support
Please email [micael@jarniac.com][mailmicael] for support.

## Roadmap
- [x] Enhance code style
- [x] Simplify communication protocol
- [ ] Implement function to broadcast variables size
- [ ] Implement function to broadcast variables name

## Contributing
Submitted code must match the [GNOME C Coding Style][gnomestyle].

## Credit
Based on the [example code][ticode] by Texas Instruments, licensed under BSD.

Maintained by [Micael Jarniac][githubmicael].

<!-- Description -->
[launchpad]: http://www.ti.com/tool/MSP-EXP430G2 "MSP-EXP430G2 LaunchPad"
[stepperdriver]: http://www.ti.com/tool/BOOST-DRV8711 "BOOST-DRV8711 BoosterPack"

<!-- Installation -->
[ccs]: http://www.ti.com/tool/CCSTUDIO "Code Composer Studio"
[ccsdown]: http://software-dl.ti.com/ccs/esd/documents/ccs_downloads.html#code-composer-studio-version-9-downloads "Code Composer Studio download page"

<!-- Usage -->
[template]: MSG_TEMPLATE.md "MSG_TEMPLATE.md file"
[steppercomms]: https://github.com/MicaelJarniac/StepperComms "MicaelJarniac/StepperComms"

<!-- Support -->
[mailmicael]: mailto:micael@jarniac.com "micael@jarniac.com"

<!-- Contributing -->
[gnomestyle]: https://developer.gnome.org/programming-guidelines/stable/c-coding-style.html.en "GNOME C Coding Style"

<!-- Credit -->
[ticode]: http://www.ti.com/lit/zip/slvc575 "Example Code"
[githubmicael]: https://github.com/MicaelJarniac
