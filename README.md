## Обратная задача кинематики

Функция __inverseKinematics__ используется для решения обратной задачи кинематики. Принимает 3 параметра: координаты точки, в которую необходимо привести клешню манипулятора.  
Функция __goBack__ позволяет осуществить возвращение клешни манипулятора в начальное положение (максимум по y и x, 0 по z). Параметров не принимает.  
Функция __calculateCoords__ рассчитывает текущие координаты клешни манипулятора на основании углов поворота плеч робота. Принимает 3 параметра: переменные, в которые будут записаны координаты x, y и z соответственно.  
Функция __printCoords__ печатает текущие координаты в USB порт.  
  
Функции __configureFirstMotor, configureSecondMotor, configureThirdMotor__ не принимают параметров и отвечают за начальную настройку каждого из двигателей.  
Функции __doFirstMotorSteps, doSecondMotorSteps, doThirdMotorSteps__ принимают 1 параметр - количество шагов и используются для совершения шагов конкретным двигателем. Отрицательное значение отвечает за вращение двигателя по часовой стрелке, положительно значение соответственно против часовой.  
  
Манипулятор работает в пределах углов, заданных в коде. Для изменения рабочих углов манипулятора необходимо заменить значения переменных __maxFirstAngle, minFirstAngle, maxSecondAngle, minSecondAngle, maxThirdAngle, minThirdAngle__.  
Более подробная информация имеется в комментариях к коду.  
Значения __L1__ и __L2__ отвечают за длину 1 и 2 плеч манипулятора соответственно.  

Для изменения начального (нулевого) положения клешни нужно заменить переменные __firstLength, secondLength, thirdLength__ на соответствующие им значения. Эти значения - количество шагов каждого двигателя от текущей точки до нулевой.  
