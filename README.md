# BME280_driver

Add below commands to /boot/config.txt

dtparam=i2c_arm=on
dtoverlay=i2c-sensor,bme280,addr=0x77

It results in adding dts overlay with the following entry to the Linux device tree:

fragment@0 {
		target = <&i2c_arm>;
		__dormant__ {
			#address-cells = <1>;
			#size-cells = <0>;
			status = "okay";

			bme280: bme280@76 {
				compatible = "bosch,bme280";
				reg = <0x77>;
				status = "okay";
			};
		};
	};
