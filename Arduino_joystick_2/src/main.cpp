#include <Arduino.h>
#include <Servo.h>

Servo servo1;  // servo de abajo (base)
Servo servo2;  // servo derecho
Servo servo3;  // servo izquierdo vertical
Servo servo4; //Servo gripper

// pines de los servos
const int pin_servo1 = 7;
const int pin_servo2 = 6;
const int pin_servo3 = 5;

// pin para estado de garra 
const int pin_garra = 4;

void setup() {

    Serial.begin(115200);  // igual que en ROS

    servo1.attach(pin_servo1);
    servo2.attach(pin_servo2);
    servo3.attach(pin_servo3);
    servo4.attach(pin_garra);

    // posiciones iniciales
    servo1.write(90);
    servo2.write(90);
    servo3.write(90);
    servo4.write(0);
}

// ángulos actuales
int ang1 = 90, ang2 = 90, ang3 = 90, ang4 = 45;

const int paso = 10;

void loop() {
    if (Serial.available() > 0) {

        String linea = Serial.readStringUntil('\n');
        linea.trim();
        if (linea.length() == 0) return;

        int c1 = linea.indexOf(',');
        int c2 = linea.indexOf(',', c1 + 1);
        int c3 = linea.indexOf(',', c2 + 1);
        if (c1 == -1 || c2 == -1 || c3 == -1) return;

        String s_eje_der      = linea.substring(0, c1);
        String s_eje_izq_ver  = linea.substring(c1 + 1, c2);
        String s_eje_der_ver  = linea.substring(c2 + 1, c3);
        String s_gripper      = linea.substring(c3 + 1);

        float eje_der     = s_eje_der.toFloat();
        float eje_izq_ver = s_eje_izq_ver.toFloat();
        float eje_der_ver = s_eje_der_ver.toFloat();
        int gripper_state = s_gripper.toInt();

        // Añadir incremento según el eje
        if (eje_der > 0.2)  ang1 += paso;
        if (eje_der < -0.2) ang1 -= paso;

        if (eje_izq_ver > 0.2)  ang2 += paso;
        if (eje_izq_ver < -0.2) ang2 -= paso;

        if (eje_der_ver > 0.2)  ang3 += paso;
        if (eje_der_ver < -0.2) ang3 -= paso;

        // límite de seguridad
        ang1 = constrain(ang1, 0, 180);
        ang2 = constrain(ang2, 0, 180);
        ang3 = constrain(ang3, 0, 180);

        // mover servos 
        servo1.write(ang1);
        servo2.write(ang2);
        servo3.write(ang3);

        // control garra simple
        servo4.write(gripper_state == 1 ? 45 : 0);

        delay(20); 
    }
}
