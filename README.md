Este proyecto tiene la finalidad de eliminar la lentitud de los UPS al activar la proteccion contra fluctuaciones de voltaje, como por ejemplo el Ecoflow River 3 Plus, que puede ser un buen UPS con un cambio de red a bateria de 10ms (segun Ecoflow), pero un rango de proteccion contra fluctuaciones de 80v a 170v, o sea, activa el UPS cuando la red baja de 80v, que puede que no sea suficiente para fuentes de computadoras sensibles como mi caso (MSI MPG A850GS). Aun asi, hay UPS con rangos de voltajes mas estrictos, pero pueden pasar varios ciclos antes de activarse.

En teoria, este protector de voltaje tiene un tiempo de reaccion de 1ms, detectando picos o bajones de tension casi instantaneamente, dejando el relay como tiempo reaccion real.

Para este proyecto use un ESP32-S, es importante que sea de 2 núcleos si quieres que tenga pantalla o comunicación serial para dejar 1 núcleo exclusivo para el sensor de voltaje.

Componentes para el protector
- Esp32s3 
- ZMPT101B
- Cualquier fuente 12v (o 5v depende)
- Reductor DC-DC configurado de 12v a 5v (opcional)
- LCD 16x2 (opcional)

Si no quieres comunicación ni pantalla LCD puede ser el ESP32 de 1 núcleo pero tienes que cambiar un poco el codigo y quitar el FreeRTOS.
En mi caso uso 12v porque en el modulo de relay (de Arduino), reemplace el relay por uno de 12v ya que me dio menor tiempo de apertura y es una mejor marca. También utilizo un optoacoplador para activar el relay.

El código puede que no sea perfecto, ya que no se mucho de C++, pero me ha funciona el 100% de las veces con las fluctuaciones de tension.

Por ahora es una idea por si sabes de microcontroladores y no una guía detallada, solo asegúrate que el microcontrolador que usaras tenga un ADC con una lectura superior de 960/s.

<img width="1170" height="1331" alt="IMG_2564" src="https://github.com/user-attachments/assets/ae8cfe7f-807b-4da2-8695-90086ec05de4" />

<img width="1170" height="1331" alt="IMG_2564" src="https://github.com/user-attachments/assets/859c6782-4ccd-4c18-913c-7139b83ef326" />

<img width="1170" height="1712" alt="IMG_2565" src="https://github.com/user-attachments/assets/74ca4eea-f88c-417a-b408-c7a50f09d173" />

<img width="3024" height="4032" alt="IMG_2561" src="https://github.com/user-attachments/assets/05ad8364-f81a-4dce-9a95-a4e54d610762" />



