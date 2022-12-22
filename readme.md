# Комп'ютерний проєкт на тему "Пошук найкоротшого шляху між двома точками поверхні". Звіт
### Команда 11

Для виконання цього завдання, учасники нашої команди розробили алгоритм, що базується на принципах дикретної математики, а саме на *алгоритмі Дейкстри*. 

### Опис проєкту

**Короткий алгоритм програми:**
* Зчитування інформації з файлу за його шляхом;
* 

#### Функція read_from_csv()
**Опис**
Функція зчитує з файлу наш граф, крок(відстань між двома сусідніми точками в сітці графу), координати початкової і кінцевої точок, між якими потрібно знайти найкоротший шлях


#### Функція get_neighbors()


#### Функція dijkstra_algorithm_optimized()
**Вхідні змінні:**
* grid - відформатована матриція зі значеннями висот точок;
* step - відстані між двома сусідніми точками в сітці графу;
* start - координати початкової точки (розташування у матриці)
* goal - координати точки, до якої маємо прийти

**Що повертає:**
* reconst_path - список кортежів, що зображають найкоротший шлях від точки до точки

**Опис**

**Внутрішні змінні:**
* queue - 
* distance_to_start - словник, що містить в собі інформацію про відомі відстані до початкової точки
* parents - словник, що показує вершину, що передує даній в шляху
* current_distance - найкоротша відстнань від точки, яку ми перевіряєио в коді до початку
* current_vertex - теперішня вершина, у якій знаходиться програма ?????
* neighbour - сусіди
* reconst_path - список, те, що програма повертає, тобто відтворений шлях від стартової точки до кінцевої

**Принцип роботи**








