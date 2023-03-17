# Итоговый отчёт команды "Ветер67".

Первый и второй дни:

1. Разработка устройства для сброса капсул:

• Устройство сбрасывает капсулы по командам "0" или "1", отправленным в терминал. На "0" сбрасываются капсулы-кубы. На "1" сбрасываются капсулы-цилиндры. Сразу после подачи команды сервопривод поворачивается на необходимый угол, отверстие диска оказывается под необходимой капсулой, и она выпадает, после диск поварачивается в своё начальное положение и находится в нём до следующей команды. При следующем сбросе капсулы такого типа угол поворота будет увеличен на угол, занимаемый одной капсулой.

<img src="https://github.com/Cobbet2/Veter67_nto_lr_2023/blob/main/image%20(2).png"  width="400">
Барабан, после модернизации
<img src="https://github.com/Cobbet2/Veter67_nto_lr_2023/blob/main/image%20(3).png"  width="400">
Крепление
<img src="https://github.com/Cobbet2/Veter67_nto_lr_2023/blob/main/image%20(4).png"  width="400">
Диск
<img src="https://github.com/Cobbet2/Veter67_nto_lr_2023/blob/main/image%20(8).png"  width="400">
Шестерня малая

<img src="https://github.com/Cobbet2/Veter67_nto_lr_2023/blob/main/image%20(5).png"  width="400">
Устройство в сборе, вид снизу
<img src="https://github.com/Cobbet2/Veter67_nto_lr_2023/blob/main/image%20(6).png"  width="400">
Устройство в сборе, вид снизу сбоку
<img src="https://github.com/Cobbet2/Veter67_nto_lr_2023/blob/main/image%20(7).png"  width="400">
Устройство в сборе, вид сверху сбоку
<img src="https://github.com/Cobbet2/Veter67_nto_lr_2023/blob/main/image%20(9).png"  width="400">

<img src="https://github.com/Cobbet2/Veter67_nto_lr_2023/blob/main/image%20(10).png"  width="400">

Крепление надевается на нижнюю деку путём отгибания лапок. Барабан плотно вставляется в обух крепления;

• Были сделаны 3D модели в Inventor всех частей, написана инструкция по эксплуатации;

Ссылка на гугл-диск: https://drive.google.com/drive/folders/1N5kGRWpcdRjvHfgeRqLB1izpUMjpMBw5

• Были напечатаны на принтере барабан, крепление, диск и шестерни.

1. Разработка программного обеспечения:

• Квадрокоптер был настроен через QGroundControl;

• Была написана программа для взлёта и посадки квадрокоптера и загружена на GitHub;

•Писали логику определения стен через маски;

• Настроили RaspberryPI.

Ссылка на гитхуб: https://github.com/Cobbet2/Veter67\_nto\_lr\_2023/blob/main/day1-2.py

Третий день:

1. Доработка устройства:

• Собрали, убедились в работоспособности идеи;

• Был доработан барабан: в целях уменьшения массы в нём при помощи паяльника были проделаны отверстия по всей боковой стенке и внутренним перегородкам;

• Сервопривод был подключен к RaspberryPI;

• Составлен алгоритм программы.


Для использования известных нам библиотек для сервопривода используется ШИМ. Так как ШИМ величина абсолютная, то надо было высчитать, какая ШИМ нужна для поворота на угол сброса куба и угол сброса цилиндра. Мы это сделали эксперементально. По алгоритму была написана и отлажена программа на Python для работы устройства.

Алгоритм программы:

счётчик цилиндров (СЦ)=0 #фактически, эти счётчики - это номера сбасываемого предмета

счётчик кубов (СК)=0

поворот в середину

функция сброса (команда):

если команда равна 0:
  
СК+=1
    
повернуться(угол=ШИМ для 90 градусов-ШИМ для поворота на угол сброса одного куба\*СК)
    
если команда равна 1:
  
СЦ+=1
    
повернуться(угол=ШИМ для поворота на угол сброса одного цилиндра\*СЦ+ШИМ 90 градусов)
    
повернуться в середину

цикл сброса:

вводится команда "1" или "0", в зависимости от того, что надо сбросить
  
запуск функции(передаваемый параметр - команда)
  
1. Доработка программного обеспечения:

• Написали логику автономного полета в помещении, избегания столкновения со стенами;

• Написали логику определения местоположения пострадавших и возгораний;

• Написали код перемещения в начало координат, вход и выход из здания.

Ссылка на гитхуб: https://github.com/Cobbet2/Veter67\_nto\_lr\_2023/blob/main/day3.py

Четвёртый день:

1) Написали итоговый отчёт.

2) Финальные отладки работы технического устройства.

3)Устранение неполадок с квадрокоптером.
