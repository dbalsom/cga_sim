# simcga

This is a digital logic simulation of the IBM CGA card, for the excellent [Digital](https://github.com/hneemann/Digital) digital logic simulator by Helmut Neemann. 

<img src="./img/screenshot_01.png" width="800" />

# Why?

I wanted to study the CGA card to better understand its operation and the operation of undefined mode flag combinations. It was also a good exercise to learn Verilog.

# Limitations

 - Currently, only text mode is implemented, although attributes work.
 - The CPU-side interface is not implemented. You have to manually load data into VRAM and toggle the bits you want in the mode control and color control registers.
 - The simulation comes with the splash screen for the incredible PC demo [Area 5150](https://www.pouet.net/prod.php?which=91938) loaded into VRAM and light blue in the CC register like the demo sets it.

# Important Note

Digital has a "VGA" output component, although the built-in timings are not compatible with CGA so I had to patch Digital to add support for CGA.
You can find the relevant changes in this branch of my fork of Digital [here](https://github.com/dbalsom/Digital_videomod/tree/cga-video-modes).

Please don't ask me how to build it. I've never built Digital, I just patched the JAR. The fork is provided for GPL3 compliance. 

# How to Use

Install the [v0.31 release of Digital](https://github.com/hneemann/Digital/releases/tag/v0.31). Download the jar from this repo's releases and copy it over the jar in the installation.

Then, open 'main.dig'.  Press the play button to start the simulation. It may take a minute for the simulation to produce a frame. A window will popup when the simulator has acquired sync lock on the output signal. You may need to resize this window to see the entire output. 

You can load video memory with different things. 
- Locate the MCM4517 component in the "Video Memory" box at bottom-center. 
- Right click on the MCM4517 and click *open*.
- Right click on the 'EEPROM' component and click *Edit*.
- Go to File, Load, and select a new VRAM dump. The file should be a 16KB binary. Note: CGA VRAM organization may not be what you expect - odd bytes from the CPU perspective are offset by 4096. 
  - You can format linear dumps of CGA VRAM to the CGA's internal format using the provided [cga_swizzle](./vram/cga_swizzle.py) script.
  - Some ready-made VRAM images are supplied in [vram](vram/).

Note if you want to change the overscan color you will need to set a new color in the Color Control register directly to the right of the CRTC. 
There is a "palette" of tunnels at the top of the diagram - copy one of the tunnels and attach it to the splitter that feeds the CC register.

# Shout outs

Thanks to *hkzlab* for redrawing all the [CGA schematics in KiCad](https://github.com/hkzlab/CGA_Schematics) which was very helpful.

Thanks to *Trixter*, *reenigne*, *VilerR*, *PCRetroTech* and the rest of the crew

Thanks to *John Elliott* for his excellent [CGA Notes](https://www.seasip.info/VintagePC/cga.html)


# License

This isn't really source code - I guess just give me attribution if you use this for anything according to the MIT license.

Everything related to [Digital](https://github.com/hneemann/Digital) retains the GPL3 license.
