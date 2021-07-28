Send My Sensor
##############

This project uses `OpenHaystack <https://github.com/seemoo-lab/openhaystack>`_ to upload sensor data from a device without internet connection by (ab)using Apple's `Find My <https://developer.apple.com/find-my/>`_ network. The data is broadcasted via Bluetooth Low Energy and forwarded by nearby Apple devices.

The application is based on the `Send My <https://github.com/positive-security/send-my>`_ project and runs on the real-time operating system `Zephyr <https://www.zephyrproject.org/>`_. Because this firmware is based on Zephyr, you can create your own sensor boards with one of the many Bluetooth Low Energy devices that Zephyr supports.

After flashing the firmware to your device, it sends out Bluetooth Low Energy advertisements with sensor data that will be visible in Send My's DataFetcher application in macOS.

Disclaimer
**********

The firmware is just a proof-of-concept and the messages that it sends can be read by anyone who knows the modem ID; there's no extra encryption or authentication.

There is also no power management yet. So if you're running this firmware on a battery-powered device, it won't be as energy-efficient as possible. If you want to improve this, all patches are welcome.

The proof of concept reads the temperature from a Bosch BME280 sensor, but it can be easily adapted to use any other sensor that Zephyr supports.

Requirements
************

* A Bluetooth Low Energy device, supported by Zephyr
* A BME280 sensor, built-in (as in the RuuviTag) or on a breakout board connected to your board's I²C bus 
* A `Zephyr development environment <https://docs.zephyrproject.org/latest/getting_started/index.html>`_
* Send My's DataFetcher application on macOS to read the sensor data

Initialization
**************

To build this firmware, you first need the `OpenHaystack Zephyr module <https://github.com/koenvervloesem/openhaystack-zephyr>`_. Install this by initializing a workspace folder (for instance ``zephyr-workspace``) where the application and all Zephyr modules will be cloned.

.. code-block:: shell

  # Initialize Zephyr workspace folder for the application (main branch)
  west init -m https://github.com/koenvervloesem/openhaystack-zephyr --mr main zephyr-workspace
  # Update Zephyr modules
  cd zephyr-workspace
  west update

Then clone the Send My Sensor repository and enter its directory:

.. code-block:: shell

  git clone https://github.com/koenvervloesem/send-my-sensor
  cd send-my-sensor

Build
*****

To build the firmware, run:

.. code-block:: shell

   west build -p auto -b $BOARD -s app

Replace ``$BOARD`` by your target board.

A sample debug configuration to read logs from the USB UART is also provided. You can apply it by running:

.. code-block:: shell

  west build -p auto -b $BOARD -s app -- -DOVERLAY_CONFIG=debug-usb-uart.conf

This only works with boards that support this, such as Nordic Semiconductor's nRF52840 Dongle.

Once you have built the application, the firmware image is available in ``build/zephyr``.

Change the modem ID
*******************

You need to specify a modem ID in the firmware image. Change the 32-bit unsigned integer value ``modem_id`` in `main.c <https://github.com/koenvervloesem/send-my-sensor/blob/main/app/src/main.c>`_) and then build the firmware.

Flash
*****

How to flash the image to a device depends on the device and its bootloader. For many devices, including the RuuviTag if you use the RuuviTag Development Kit, you can run:

.. code-block:: shell

   west flash

Refer to your `board's documentation <https://docs.zephyrproject.org/latest/boards/index.html>`_ for alternative flash instructions if your board doesn't support the ``flash`` target.

For the nRF52840 Dongle with the built-in bootloader, run:

.. code-block:: shell

  nrfutil pkg generate --hw-version 52 --sd-req=0x00 \
          --application build/zephyr/zephyr.hex \
          --application-version 1 send-my-sensor.zip

This packages the application in the file ``send-my-sensor.zip``. Now press the reset button and flash the package onto the board with:

.. code-block:: shell

  nrfutil dfu usb-serial -pkg send-my-sensor.zip -p /dev/ttyACM0

Have a look at ``ls /dev/tty*`` for the right device on Linux and macOS. On Windows it should be something like ``COMx``.

Supported devices
*****************

This procedure has been tested with:

* Nordic Semiconductor's `nRF52840 Dongle <https://docs.zephyrproject.org/latest/boards/arm/nrf52840dongle_nrf52840/doc/index.html>`_ (board name ``nrf52840dongle_nrf52840``) with an I²C breakout board of the Bosch BME280 temperature/humidity/pressure sensor. Solder headers to the dongle and connect the BME280's SDA to pin 0.31, SCL to pin 0.29, GND to GND and VCC to VDD. If your BME280 sensor has another I²C address than 0x76, change this in the device overlay `nrf52840dongle_nrf52840.overlay <https://github.com/koenvervloesem/send-my-sensor/blob/main/app/nrf52840dongle_nrf52840.overlay>`_.
* Ruuvi's nRF52832-based `RuuviTag <https://docs.zephyrproject.org/latest/boards/arm/ruuvi_ruuvitag/doc/index.html>`_ (board name ``ruuvi_ruuvitag``), which has a built-in BME280. Use the `RuuviTag Development Kit <https://ruuvi.com/products/ruuvitag-development-kit/>`_ to flash the firmware.

Other Bluetooth Low Energy devices supported by Zephyr should work as well, as long as they have a built-in BME280 or you can connect one over I²C. You may need a custom device overlay. Please let me know if you manage to run this firmware on another board, or if you need assistance, so I can add it to the list of devices it has been tested with.

Acknowledgments
***************

This project is inspired by and has used code from:

* the original `Send My firmware for ESP32 <https://github.com/positive-security/send-my/tree/main/Firmware/ESP32>`_
* the `Zephyr Example Application <https://github.com/zephyrproject-rtos/example-application>`_ for the project structure

License
*******

This project is provided by `Koen Vervloesem <http://koen.vervloesem.eu>`_ as open source software with the GNU Affero General Public License v3.0. See the `LICENSE file <LICENSE>`_ for more information.
