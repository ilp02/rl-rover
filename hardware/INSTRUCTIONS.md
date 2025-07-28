# Rover Assembly and Wiring

## 1. Motors, wheels, and chassis

* Start by assembling the motor and wheels. There is a screw for attaching the wheels to the motor axle.
* Attach the H-shaped piece to the motors, and place them on the bottom plate.
* Each H-piece has a matching hole in the plate. Fit it in.
* Use tape to hold the motors tightly in place.
* Screw in the ceramic ball holder firmly.
* Add the four columns to the holes and screw them from the bottom. Take care with the motor wires so they do not get kinked between the motors and the columns.

## 2. Motor driver and power (DRV8833 + battery)

* Connect the motor wires to the DRV8833 **OUT1–OUT4** pins. **OUT1 and OUT2** go to one motor. **OUT3 and OUT4** go to the other motor.
* After connecting the motors, attach the **positive** lead from the **3 V battery pack** to **VCC**. Leave the **negative** lead unconnected for now.
* The negative lead will act as the **common ground** that both the Raspberry Pi and the DRV8833 will share.

## 3. Common ground connector

* Use a small piece of jumper pin array to make a common ground connector.
* Solder the tops together so the pins are electrically connected.
* Connect DRV8833 **GND** and the battery **negative** lead to this connector.
* Add a third jumper from this common ground for the **Pi GND**.

## 4. Control pins to the Pi

* After the steps above, you should have **IN1–IN4**, **ULT**, and **EEP** pins left on the DRV8833.
* Ignore **ULT**.
* **IN1–IN4** and **EEP** will connect to the Pi, along with the ground jumper from the common ground.
* Connect jumpers to each of these pins and thread them up through the large holes in the top plate.
* Before joining the top and bottom plates, also thread the **rechargeable battery** cables through another hole, preferably on the right side.

## 5. Close the chassis

* Mesh both plates into the H-piece, then screw the columns together.
* Press the DRV8833 between the plates while gently pulling the jumper wires. The wires may be short, so take your time.

## 6. GPIO mapping and power rails

* Connect **IN1–IN4** to GPIO pins that support software PWM. The attached code uses **GPIO 27/17 and 22/23**.
* Connect the **common ground** to a **GND** pin on the Pi.
* Connect **EEP** to **3.3 V** on the Pi.
* **Warning:** Do not connect **EEP** to **5 V**. That will damage the DRV8833.

## 7. Mount the Pi and battery

* After wiring the Pi, attach it to the plate with insulation underneath. Thick foam stickers with adhesive(products for cars and closets) work well, and the Pi can be held by small screws into the foam.
* Hold the rechargeable battery with tape and make sure it is tight without slack.
* Connect the USB‑C charger to the Pi and check that the green LED turns on. After the Pi boots, turn on the **3 V** battery. The LED on the DRV8833 should light.

## 8. Pi setup and quick test

* Use SSH to access the Pi. Create a **virtual environment (venv)** and install **Python 3.11**.
* Run motor test script to confirm that the motors spin.
* Install the project requirements.
