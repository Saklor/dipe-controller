# Controlador de Dipes

Este controlador se hizo teniendo como objetivo el manejo de terminales subsidiarias (hasta 4) en comunicacion constante con una terminal central.

## Terminales Secundarias

Las terminales secundarias o DIPEs realizan una medicion de varios sensores y reportan su estado periodicamente a la central mediante la codificacion de 1 byte utilizando RS-485

## Terminal Primaria

La terminal primaria mantiene un registro de los estados de las terminales secundarias y avisa mediante pantalla y LEDs en caso de ocurrir cualquier falla, o de no recibir actualizaciones de estado despues de un intervalo de tiempo predeterminado.
