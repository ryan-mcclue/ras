/dts-v1/;
/memreserve/ 0x00000000 0x00001000;
/ {
 compatible = "brcm,bcm2835";
 model = "BCM2835";
 #address-cells = <1>;
 #size-cells = <1>;
 aliases {
  serial0 = &uart0;
  serial1 = &uart1;
 };
 chosen {
  stdout-path = "serial0:115200n8";
 };
 rmem: reserved-memory {
  #address-cells = <1>;
  #size-cells = <1>;
  ranges;
  cma: 1,cma {
   compatible = "shared-dma-pool";
   size = <0x4000000>;
   reusable;
   1,cma-default;
  };
 };
 thermal-zones {
  cpu_thermal: cpu-thermal {
   polling-delay-passive = <0>;
   polling-delay = <1000>;
   trips {
    cpu-crit {
     temperature = <90000>;
     hysteresis = <0>;
     type = "critical";
    };
   };
   cooling-maps {
   };
  };
 };
 soc {
  compatible = "simple-bus";
  #address-cells = <1>;
  #size-cells = <1>;
  system_timer: timer@7e003000 {
   compatible = "brcm,bcm2835-system-timer";
   reg = <0x7e003000 0x1000>;
   interrupts = <1 0>, <1 1>, <1 2>, <1 3>;
   clock-frequency = <1000000>;
  };
  txp: txp@7e004000 {
   compatible = "brcm,bcm2835-txp";
   reg = <0x7e004000 0x20>;
   interrupts = <1 11>;
  };
  clocks: cprman@7e101000 {
   compatible = "brcm,bcm2835-cprman";
   #clock-cells = <1>;
   reg = <0x7e101000 0x2000>;
   clocks = <&clk_osc>,
    <&dsi0 0>, <&dsi0 1>, <&dsi0 2>,
    <&dsi1 0>, <&dsi1 1>, <&dsi1 2>;
  };
  mailbox: mailbox@7e00b880 {
   compatible = "brcm,bcm2835-mbox";
   reg = <0x7e00b880 0x40>;
   interrupts = <0 1>;
   #mbox-cells = <0>;
  };
  gpio: gpio@7e200000 {
   compatible = "brcm,bcm2835-gpio";
   reg = <0x7e200000 0xb4>;
   interrupts = <2 17>, <2 18>, <2 19>, <2 20>;
   gpio-controller;
   #gpio-cells = <2>;
   interrupt-controller;
   #interrupt-cells = <2>;
   gpio-ranges = <&gpio 0 0 54>;
   dpi_gpio0: dpi_gpio0 {
    brcm,pins = <0 1 2 3 4 5 6 7 8 9 10 11
          12 13 14 15 16 17 18 19
          20 21 22 23 24 25 26 27>;
    brcm,function = <6>;
   };
   emmc_gpio22: emmc_gpio22 {
    brcm,pins = <22 23 24 25 26 27>;
    brcm,function = <7>;
   };
   emmc_gpio34: emmc_gpio34 {
    brcm,pins = <34 35 36 37 38 39>;
    brcm,function = <7>;
    brcm,pull = <0
          2
          2
          2
          2
          2>;
   };
   emmc_gpio48: emmc_gpio48 {
    brcm,pins = <48 49 50 51 52 53>;
    brcm,function = <7>;
   };
   gpclk0_gpio4: gpclk0_gpio4 {
    brcm,pins = <4>;
    brcm,function = <4>;
   };
   gpclk1_gpio5: gpclk1_gpio5 {
    brcm,pins = <5>;
    brcm,function = <4>;
   };
   gpclk1_gpio42: gpclk1_gpio42 {
    brcm,pins = <42>;
    brcm,function = <4>;
   };
   gpclk1_gpio44: gpclk1_gpio44 {
    brcm,pins = <44>;
    brcm,function = <4>;
   };
   gpclk2_gpio6: gpclk2_gpio6 {
    brcm,pins = <6>;
    brcm,function = <4>;
   };
   gpclk2_gpio43: gpclk2_gpio43 {
    brcm,pins = <43>;
    brcm,function = <4>;
    brcm,pull = <0>;
   };
   i2c0_gpio0: i2c0_gpio0 {
    brcm,pins = <0 1>;
    brcm,function = <4>;
   };
   i2c0_gpio28: i2c0_gpio28 {
    brcm,pins = <28 29>;
    brcm,function = <4>;
   };
   i2c0_gpio44: i2c0_gpio44 {
    brcm,pins = <44 45>;
    brcm,function = <5>;
   };
   i2c1_gpio2: i2c1_gpio2 {
    brcm,pins = <2 3>;
    brcm,function = <4>;
   };
   i2c1_gpio44: i2c1_gpio44 {
    brcm,pins = <44 45>;
    brcm,function = <6>;
   };
   jtag_gpio22: jtag_gpio22 {
    brcm,pins = <22 23 24 25 26 27>;
    brcm,function = <3>;
   };
   pcm_gpio18: pcm_gpio18 {
    brcm,pins = <18 19 20 21>;
    brcm,function = <4>;
   };
   pcm_gpio28: pcm_gpio28 {
    brcm,pins = <28 29 30 31>;
    brcm,function = <6>;
   };
   sdhost_gpio48: sdhost_gpio48 {
    brcm,pins = <48 49 50 51 52 53>;
    brcm,function = <4>;
   };
   spi0_gpio7: spi0_gpio7 {
    brcm,pins = <7 8 9 10 11>;
    brcm,function = <4>;
   };
   spi0_gpio35: spi0_gpio35 {
    brcm,pins = <35 36 37 38 39>;
    brcm,function = <4>;
   };
   spi1_gpio16: spi1_gpio16 {
    brcm,pins = <16 17 18 19 20 21>;
    brcm,function = <3>;
   };
   spi2_gpio40: spi2_gpio40 {
    brcm,pins = <40 41 42 43 44 45>;
    brcm,function = <3>;
   };
   uart0_gpio14: uart0_gpio14 {
    brcm,pins = <14 15>;
    brcm,function = <4>;
   };
   uart0_ctsrts_gpio16: uart0_ctsrts_gpio16 {
    brcm,pins = <16 17>;
    brcm,function = <7>;
   };
   uart0_ctsrts_gpio30: uart0_ctsrts_gpio30 {
    brcm,pins = <30 31>;
    brcm,function = <7>;
    brcm,pull = <2 0>;
   };
   uart0_gpio32: uart0_gpio32 {
    brcm,pins = <32 33>;
    brcm,function = <7>;
    brcm,pull = <0 2>;
   };
   uart0_gpio36: uart0_gpio36 {
    brcm,pins = <36 37>;
    brcm,function = <6>;
   };
   uart0_ctsrts_gpio38: uart0_ctsrts_gpio38 {
    brcm,pins = <38 39>;
    brcm,function = <6>;
   };
   uart1_gpio14: uart1_gpio14 {
    brcm,pins = <14 15>;
    brcm,function = <2>;
   };
   uart1_ctsrts_gpio16: uart1_ctsrts_gpio16 {
    brcm,pins = <16 17>;
    brcm,function = <2>;
   };
   uart1_gpio32: uart1_gpio32 {
    brcm,pins = <32 33>;
    brcm,function = <2>;
   };
   uart1_ctsrts_gpio30: uart1_ctsrts_gpio30 {
    brcm,pins = <30 31>;
    brcm,function = <2>;
   };
   uart1_gpio40: uart1_gpio40 {
    brcm,pins = <40 41>;
    brcm,function = <2>;
   };
   uart1_ctsrts_gpio42: uart1_ctsrts_gpio42 {
    brcm,pins = <42 43>;
    brcm,function = <2>;
   };
  };
  uart0: serial@7e201000 {
   compatible = "arm,pl011", "arm,primecell";
   reg = <0x7e201000 0x200>;
   interrupts = <2 25>;
   clocks = <&clocks 19>,
     <&clocks 20>;
   clock-names = "uartclk", "apb_pclk";
   arm,primecell-periphid = <0x00241011>;
  };
  sdhost: mmc@7e202000 {
   compatible = "brcm,bcm2835-sdhost";
   reg = <0x7e202000 0x100>;
   interrupts = <2 24>;
   clocks = <&clocks 20>;
   status = "disabled";
  };
  i2s: i2s@7e203000 {
   compatible = "brcm,bcm2835-i2s";
   reg = <0x7e203000 0x24>;
   clocks = <&clocks 31>;
   status = "disabled";
  };
  spi: spi@7e204000 {
   compatible = "brcm,bcm2835-spi";
   reg = <0x7e204000 0x200>;
   interrupts = <2 22>;
   clocks = <&clocks 20>;
   #address-cells = <1>;
   #size-cells = <0>;
   status = "disabled";
  };
  i2c0if: i2c@7e205000 {
   compatible = "brcm,bcm2835-i2c";
   reg = <0x7e205000 0x200>;
   interrupts = <2 21>;
   clocks = <&clocks 20>;
   #address-cells = <1>;
   #size-cells = <0>;
   status = "disabled";
  };
  i2c0mux: i2c0mux {
   compatible = "i2c-mux-pinctrl";
   #address-cells = <1>;
   #size-cells = <0>;
   i2c-parent = <&i2c0if>;
   pinctrl-names = "i2c0", "i2c_csi_dsi";
   status = "disabled";
   i2c0: i2c@0 {
    reg = <0>;
    #address-cells = <1>;
    #size-cells = <0>;
   };
   i2c_csi_dsi: i2c@1 {
    reg = <1>;
    #address-cells = <1>;
    #size-cells = <0>;
   };
  };
  dpi: dpi@7e208000 {
   compatible = "brcm,bcm2835-dpi";
   reg = <0x7e208000 0x8c>;
   clocks = <&clocks 20>,
     <&clocks 44>;
   clock-names = "core", "pixel";
   #address-cells = <1>;
   #size-cells = <0>;
   status = "disabled";
  };
  dsi0: dsi@7e209000 {
   compatible = "brcm,bcm2835-dsi0";
   reg = <0x7e209000 0x78>;
   interrupts = <2 4>;
   #address-cells = <1>;
   #size-cells = <0>;
   #clock-cells = <1>;
   clocks = <&clocks 32>,
     <&clocks 47>,
     <&clocks 49>;
   clock-names = "phy", "escape", "pixel";
   clock-output-names = "dsi0_byte",
          "dsi0_ddr2",
          "dsi0_ddr";
   status = "disabled";
  };
  aux: aux@7e215000 {
   compatible = "brcm,bcm2835-aux";
   #clock-cells = <1>;
   reg = <0x7e215000 0x8>;
   clocks = <&clocks 20>;
  };
  uart1: serial@7e215040 {
   compatible = "brcm,bcm2835-aux-uart";
   reg = <0x7e215040 0x40>;
   interrupts = <1 29>;
   clocks = <&aux 0>;
   status = "disabled";
  };
  spi1: spi@7e215080 {
   compatible = "brcm,bcm2835-aux-spi";
   reg = <0x7e215080 0x40>;
   interrupts = <1 29>;
   clocks = <&aux 1>;
   #address-cells = <1>;
   #size-cells = <0>;
   status = "disabled";
  };
  spi2: spi@7e2150c0 {
   compatible = "brcm,bcm2835-aux-spi";
   reg = <0x7e2150c0 0x40>;
   interrupts = <1 29>;
   clocks = <&aux 2>;
   #address-cells = <1>;
   #size-cells = <0>;
   status = "disabled";
  };
  pwm: pwm@7e20c000 {
   compatible = "brcm,bcm2835-pwm";
   reg = <0x7e20c000 0x28>;
   clocks = <&clocks 30>;
   assigned-clocks = <&clocks 30>;
   assigned-clock-rates = <10000000>;
   #pwm-cells = <2>;
   status = "disabled";
  };
  sdhci: mmc@7e300000 {
   compatible = "brcm,bcm2835-sdhci";
   reg = <0x7e300000 0x100>;
   interrupts = <2 30>;
   clocks = <&clocks 28>;
   status = "disabled";
  };
  hvs@7e400000 {
   compatible = "brcm,bcm2835-hvs";
   reg = <0x7e400000 0x6000>;
   interrupts = <2 1>;
  };
  dsi1: dsi@7e700000 {
   compatible = "brcm,bcm2835-dsi1";
   reg = <0x7e700000 0x8c>;
   interrupts = <2 12>;
   #address-cells = <1>;
   #size-cells = <0>;
   #clock-cells = <1>;
   clocks = <&clocks 35>,
     <&clocks 48>,
     <&clocks 50>;
   clock-names = "phy", "escape", "pixel";
   clock-output-names = "dsi1_byte",
          "dsi1_ddr2",
          "dsi1_ddr";
   status = "disabled";
  };
  i2c1: i2c@7e804000 {
   compatible = "brcm,bcm2835-i2c";
   reg = <0x7e804000 0x1000>;
   interrupts = <2 21>;
   clocks = <&clocks 20>;
   #address-cells = <1>;
   #size-cells = <0>;
   status = "disabled";
  };
  usb: usb@7e980000 {
   compatible = "brcm,bcm2835-usb";
   reg = <0x7e980000 0x10000>;
   interrupts = <1 9>;
   #address-cells = <1>;
   #size-cells = <0>;
   clocks = <&clk_usb>;
   clock-names = "otg";
   phys = <&usbphy>;
   phy-names = "usb2-phy";
  };
 };
 clocks {
  clk_osc: clk-osc {
   compatible = "fixed-clock";
   #clock-cells = <0>;
   clock-output-names = "osc";
   clock-frequency = <19200000>;
  };
  clk_usb: clk-usb {
   compatible = "fixed-clock";
   #clock-cells = <0>;
   clock-output-names = "otg";
   clock-frequency = <480000000>;
  };
 };
 usbphy: phy {
  compatible = "usb-nop-xceiv";
  #phy-cells = <0>;
 };
};
/ {
 interrupt-parent = <&intc>;
 soc {
  dma: dma@7e007000 {
   compatible = "brcm,bcm2835-dma";
   reg = <0x7e007000 0xf00>;
   interrupts = <1 16>,
         <1 17>,
         <1 18>,
         <1 19>,
         <1 20>,
         <1 21>,
         <1 22>,
         <1 23>,
         <1 24>,
         <1 25>,
         <1 26>,
         <1 27>,
         <1 27>,
         <1 27>,
         <1 27>,
         <1 28>;
   interrupt-names = "dma0",
       "dma1",
       "dma2",
       "dma3",
       "dma4",
       "dma5",
       "dma6",
       "dma7",
       "dma8",
       "dma9",
       "dma10",
       "dma11",
       "dma12",
       "dma13",
       "dma14",
       "dma-shared-all";
   #dma-cells = <1>;
   brcm,dma-channel-mask = <0x7f35>;
  };
  intc: interrupt-controller@7e00b200 {
   compatible = "brcm,bcm2835-armctrl-ic";
   reg = <0x7e00b200 0x200>;
   interrupt-controller;
   #interrupt-cells = <2>;
  };
  pm: watchdog@7e100000 {
   compatible = "brcm,bcm2835-pm", "brcm,bcm2835-pm-wdt";
   #power-domain-cells = <1>;
   #reset-cells = <1>;
   reg = <0x7e100000 0x114>,
         <0x7e00a000 0x24>;
   clocks = <&clocks 21>,
     <&clocks 29>,
     <&clocks 23>,
     <&clocks 22>;
   clock-names = "v3d", "peri_image", "h264", "isp";
   system-power-controller;
  };
  rng@7e104000 {
   compatible = "brcm,bcm2835-rng";
   reg = <0x7e104000 0x10>;
   interrupts = <2 29>;
  };
  pixelvalve@7e206000 {
   compatible = "brcm,bcm2835-pixelvalve0";
   reg = <0x7e206000 0x100>;
   interrupts = <2 13>;
  };
  pixelvalve@7e207000 {
   compatible = "brcm,bcm2835-pixelvalve1";
   reg = <0x7e207000 0x100>;
   interrupts = <2 14>;
  };
  thermal: thermal@7e212000 {
   compatible = "brcm,bcm2835-thermal";
   reg = <0x7e212000 0x8>;
   clocks = <&clocks 27>;
   #thermal-sensor-cells = <0>;
   status = "disabled";
  };
  i2c2: i2c@7e805000 {
   compatible = "brcm,bcm2835-i2c";
   reg = <0x7e805000 0x1000>;
   interrupts = <2 21>;
   clocks = <&clocks 20>;
   #address-cells = <1>;
   #size-cells = <0>;
   status = "okay";
  };
  vec: vec@7e806000 {
   compatible = "brcm,bcm2835-vec";
   reg = <0x7e806000 0x1000>;
   clocks = <&firmware_clocks 15>;
   interrupts = <2 27>;
   status = "disabled";
  };
  pixelvalve@7e807000 {
   compatible = "brcm,bcm2835-pixelvalve2";
   reg = <0x7e807000 0x100>;
   interrupts = <2 10>;
  };
  hdmi: hdmi@7e902000 {
   compatible = "brcm,bcm2835-hdmi";
   reg = <0x7e902000 0x600>,
         <0x7e808000 0x100>;
   reg-names = "hdmi",
        "hd";
   interrupts = <2 8>, <2 9>;
   ddc = <&i2c2>;
   clocks = <&firmware_clocks 9>,
     <&clocks 25>;
   clock-names = "pixel", "hdmi";
   dmas = <&dma (17|(1<<27)|(1<<24))>;
   dma-names = "audio-rx";
   status = "disabled";
  };
  v3d: v3d@7ec00000 {
   compatible = "brcm,bcm2835-v3d";
   reg = <0x7ec00000 0x1000>;
   interrupts = <1 10>;
  };
  vc4: gpu {
   compatible = "brcm,bcm2835-vc4";
  };
 };
};
&cpu_thermal {
 thermal-sensors = <&thermal>;
};
&gpio {
 i2c_slave_gpio18: i2c_slave_gpio18 {
  brcm,pins = <18 19 20 21>;
  brcm,function = <7>;
 };
 jtag_gpio4: jtag_gpio4 {
  brcm,pins = <4 5 6 12 13>;
  brcm,function = <2>;
 };
 pwm0_gpio12: pwm0_gpio12 {
  brcm,pins = <12>;
  brcm,function = <4>;
 };
 pwm0_gpio18: pwm0_gpio18 {
  brcm,pins = <18>;
  brcm,function = <2>;
 };
 pwm0_gpio40: pwm0_gpio40 {
  brcm,pins = <40>;
  brcm,function = <4>;
 };
 pwm1_gpio13: pwm1_gpio13 {
  brcm,pins = <13>;
  brcm,function = <4>;
 };
 pwm1_gpio19: pwm1_gpio19 {
  brcm,pins = <19>;
  brcm,function = <2>;
 };
 pwm1_gpio41: pwm1_gpio41 {
  brcm,pins = <41>;
  brcm,function = <4>;
 };
 pwm1_gpio45: pwm1_gpio45 {
  brcm,pins = <45>;
  brcm,function = <4>;
 };
};
&i2s {
 dmas = <&dma 2>, <&dma 3>;
 dma-names = "tx", "rx";
};
&sdhost {
 dmas = <&dma 13>;
 dma-names = "rx-tx";
};
&spi {
 dmas = <&dma 6>, <&dma 7>;
 dma-names = "tx", "rx";
};
&v3d {
 power-domains = <&power 10>;
};
/ {
 compatible = "brcm,bcm2836";
 soc {
  ranges = <0x7e000000 0x3f000000 0x1000000>,
    <0x40000000 0x40000000 0x00001000>;
  dma-ranges = <0xc0000000 0x00000000 0x3f000000>;
  local_intc: local_intc@40000000 {
   compatible = "brcm,bcm2836-l1-intc";
   reg = <0x40000000 0x100>;
   interrupt-controller;
   #interrupt-cells = <2>;
   interrupt-parent = <&local_intc>;
  };
 };
 arm-pmu {
  compatible = "arm,cortex-a7-pmu";
  interrupt-parent = <&local_intc>;
  interrupts = <9 4>;
 };
 timer {
  compatible = "arm,armv7-timer";
  interrupt-parent = <&local_intc>;
  interrupts = <0 4>,
        <1 4>,
        <3 4>,
        <2 4>;
  always-on;
 };
 cpus: cpus {
  #address-cells = <1>;
  #size-cells = <0>;
  enable-method = "brcm,bcm2836-smp";
  v7_cpu0: cpu@0 {
   device_type = "cpu";
   compatible = "arm,cortex-a7";
   reg = <0xf00>;
   clock-frequency = <800000000>;
  };
  v7_cpu1: cpu@1 {
   device_type = "cpu";
   compatible = "arm,cortex-a7";
   reg = <0xf01>;
   clock-frequency = <800000000>;
  };
  v7_cpu2: cpu@2 {
   device_type = "cpu";
   compatible = "arm,cortex-a7";
   reg = <0xf02>;
   clock-frequency = <800000000>;
  };
  v7_cpu3: cpu@3 {
   device_type = "cpu";
   compatible = "arm,cortex-a7";
   reg = <0xf03>;
   clock-frequency = <800000000>;
  };
 };
};
&intc {
 compatible = "brcm,bcm2836-armctrl-ic";
 reg = <0x7e00b200 0x200>;
 interrupt-parent = <&local_intc>;
 interrupts = <8 4>;
};
&cpu_thermal {
 coefficients = <(-538) 407000>;
};
&thermal {
 compatible = "brcm,bcm2836-thermal";
 status = "okay";
};
/ {
 leds {
  compatible = "gpio-leds";
  led-act {
   label = "ACT";
   default-state = "keep";
   1,default-trigger = "heartbeat";
  };
 };
 soc {
  firmware: firmware {
   compatible = "raspberrypi,bcm2835-firmware", "simple-mfd";
   #address-cells = <1>;
   #size-cells = <1>;
   mboxes = <&mailbox>;
   dma-ranges;
   firmware_clocks: clocks {
    compatible = "raspberrypi,firmware-clocks";
    #clock-cells = <1>;
   };
  };
  power: power {
   compatible = "raspberrypi,bcm2835-power";
   firmware = <&firmware>;
   #power-domain-cells = <1>;
  };
  vchiq: mailbox@7e00b840 {
   compatible = "brcm,bcm2835-vchiq";
   reg = <0x7e00b840 0x3c>;
   interrupts = <0 2>;
  };
 };
};
&gpio {
 pinctrl-names = "default";
 gpioout: gpioout {
  brcm,pins = <6>;
  brcm,function = <1>;
 };
 alt0: alt0 {
  brcm,pins = <4 5 7 8 9 10 11>;
  brcm,function = <4>;
 };
};
&i2c0if {
 status = "okay";
 clock-frequency = <100000>;
};
&i2c0mux {
 pinctrl-0 = <&i2c0_gpio0>;
 status = "okay";
};
&i2c1 {
 pinctrl-names = "default";
 pinctrl-0 = <&i2c1_gpio2>;
 status = "okay";
 clock-frequency = <100000>;
};
&usb {
 power-domains = <&power 6>;
};
&vc4 {
 raspberrypi,firmware = <&firmware>;
};
&vec {
 power-domains = <&power 7>;
 status = "okay";
};
&dsi0 {
 power-domains = <&power 17>;
};
&dsi1 {
 power-domains = <&power 18>;
};
&vchiq {
 compatible = "brcm,bcm2836-vchiq", "brcm,bcm2835-vchiq";
};
/ {
 aliases {
  ethernet0 = &ethernet;
 };
};
&usb {
 usb1@1 {
  compatible = "usb424,9514";
  reg = <1>;
  #address-cells = <1>;
  #size-cells = <0>;
  ethernet: usbether@1 {
   compatible = "usb424,ec00";
   reg = <1>;
  };
 };
};
&usb {
 dr_mode = "host";
};
/ {
 compatible = "raspberrypi,2-model-b", "brcm,bcm2836";
 model = "Raspberry Pi 2 Model B";
 memory@0 {
  device_type = "memory";
  reg = <0 0x40000000>;
 };
 leds {
  led-act {
   gpios = <&gpio 47 0>;
  };
  led-pwr {
   label = "PWR";
   gpios = <&gpio 35 0>;
   default-state = "keep";
   1,default-trigger = "default-on";
  };
 };
};
&gpio {
 gpio-line-names = "ID_SDA",
     "ID_SCL",
     "SDA1",
     "SCL1",
     "GPIO_GCLK",
     "GPIO5",
     "GPIO6",
     "SPI_CE1_N",
     "SPI_CE0_N",
     "SPI_MISO",
     "SPI_MOSI",
     "SPI_SCLK",
     "GPIO12",
     "GPIO13",
     "TXD0",
     "RXD0",
     "GPIO16",
     "GPIO17",
     "GPIO18",
     "GPIO19",
     "GPIO20",
     "GPIO21",
     "GPIO22",
     "GPIO23",
     "GPIO24",
     "GPIO25",
     "GPIO26",
     "GPIO27",
     "SDA0",
     "SCL0",
     "",
     "LAN_RUN",
     "CAM_GPIO1",
     "",
     "",
     "PWR_LOW_N",
     "",
     "",
     "USB_LIMIT",
     "",
     "PWM0_OUT",
     "CAM_GPIO0",
     "SMPS_SCL",
     "SMPS_SDA",
     "ETHCLK",
     "PWM1_OUT",
     "HDMI_HPD_N",
     "STATUS_LED",
     "SD_CLK_R",
     "SD_CMD_R",
     "SD_DATA0_R",
     "SD_DATA1_R",
     "SD_DATA2_R",
     "SD_DATA3_R";
 pinctrl-0 = <&gpioout &alt0 &i2s_alt0>;
 i2s_alt0: i2s_alt0 {
  brcm,pins = <18 19 20 21>;
  brcm,function = <4>;
 };
};
&hdmi {
 hpd-gpios = <&gpio 46 1>;
 power-domains = <&power 5>;
 status = "okay";
};
&pwm {
 pinctrl-names = "default";
 pinctrl-0 = <&pwm0_gpio40 &pwm1_gpio45>;
 status = "okay";
};
&sdhost {
 pinctrl-names = "default";
 pinctrl-0 = <&sdhost_gpio48>;
 bus-width = <4>;
 status = "okay";
};
&uart0 {
 pinctrl-names = "default";
 pinctrl-0 = <&uart0_gpio14>;
 status = "okay";
};
&i2c0mux {
 pinctrl-1 = <&i2c0_gpio28>;
};
