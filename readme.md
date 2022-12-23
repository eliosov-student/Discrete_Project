# Комп'ютерний проєкт на тему "Пошук найкоротшого шляху між двома точками поверхні". Звіт
### Команда 11

Для виконання цього завдання, учасники нашої команди розробили алгоритм, що базується на принципах диcкретної математики, а саме на *алгоритмі Дейкстри*. 

### Опис проєкту:
Проєкт містить алгоритм, який знаходить найкоротший шлях між двома заданими точками прямокутної ділянки ландшафту. Ця ділянка - це прямокутна сітка точок з заданими в них висотами.

#### Функція read_from_csv()
**Вхідна змінна:**

* Назва файлу, у якому міститься все, що потрібне для тестування програми

**Що повертає:**

* graph - список списків з усіма точками(їх висотами)
* step - відстань між двома сусідніми точками в сітці графу
* start - координата початкової точки
* goal - координата кінцевої точки

**Опис:**

Функція зчитує з файлу наш граф, крок(відстань між двома сусідніми точками в сітці графу), координати початкової і кінцевої точок, між якими потрібно знайти найкоротший шлях


#### Функція get_neighbors()
**Вхідні змінні:**

* grid - список списків з усіма вершинами(їх висотами)
* vertex - вершина, задана координатами

**Що повертає:**

* neighbors - список списків, у яких є координати сусідів vertex та висота, на яких вони розташовані

**Опис:**

Функція шукає сусідні вершини для точки vertex

**Принцип роботи:**

Ми шукаємо сусідні вершини для точки vertex по горизонталі та вертикалі
* if vertex[0] > 0 : цією умовою перевіряємо, чи є зверху сусідня точка
* if vertex[1] > 0 : цією умовою перевіряємо, чи є зліва сусідня точка
* if vertex[0] < length : цією умовою перевіряємо, чи є знизу сусідня точка 
* if vertex[1] < length : цією умовою перевіряємо, чи є справа сусідня точка

Якщо такі точки є, то ми додаємо їх у наш список neighbors y вигляді списку, у якому знаходяться їх координати та висоти, на яких вони розташовані

#### Функція dijkstra_algorithm_optimized()
**Вхідні змінні:**
* grid - список списків з усіма точками(їх висотами)
* step - відстань між двома сусідніми точками в сітці графу
* start - координата початкової точки
* goal - координата кінцевої точки

**Що повертає:**
* reconst_path - список кортежів, що зображають найкоротший шлях від точки до точки

**Опис:**

**Внутрішні змінні:**
* queue - черга з вершин, які алгоритм індентифікував, але ще не розглядав
* distance_to_start - словник, що містить в собі інформацію про відомі відстані до початкової точки
* parents - словник, що показує вершину, що передує даній в шляху
* current_distance - найкоротша відстнань від точки, яку ми перевіряєио в коді до початку
* current_vertex - вершина, що зараз розглядається
* neighbour - сусіди
* reconst_path - список, те, що програма повертає, тобто відтворений шлях від стартової точки до кінцевої

**Принцип роботи:**

Основний алгоритм відбувається в циклі *while queue*.
З допомогою модуля **heapq** з queue прибирається вершина з найменшою відстанню до початкової і вона береться на розгляд.

Поки кінцевої точки(goal) не досягнуто, цикл діє таким чином:
Завдяки виклику функції get_neighbors ми отримуємо сусідні вершини до точки (current_vertex). 
Через цикл *for* ми отримуєсо відстані від точки у якій знаходимося до кожної сусідньої.
Якщо якась з сусідніх точок вже є в словнику, а отримана відстань менша за ту, що вже була, або вершина взагалі використовується вперше, то вершина додається в *queue*, до того ж *queue* лишається впорядкованою, а у словник відстаней (distance_to_start) додається відстань до цієї точки.

Далі за *current_vertex* приймається вершина з queue, що має найменшу відстань до старту. У випадку, якщо взятий найменший елемент більше не є актуальним, ми знову повертаємося до queue і беремо наступне значення за тим самим принципом.

Цикл повторюється доки не буде досягнуто кінцевої вершини (goal). В такому випадку цикл переривається і з допомогою циклу *while* в список заноситься відтворений найкоротший шлях від стартової вершини до кінцевої.


### Хід роботи:

* Обговорення в команді;
* Написання допоміжних функцій;
* Написання А*;
* Написання алгоритму Дейкстри;
* Вибір Дейкстри;
* Оптимізація;
* Тестування;
* Написання звіту;
* Створення презентації.


### Розподіл обов'язків в команді:

| Учасник | Опис роботи |
| --- | --- |
| Михайло Еліосов | Написанння коду програми |
| Артем Краєвський | Написання коду програми |
| Софія Шуляк | Тестування та написання звіту |
| Анастасія Диня | Тестування та написання звіту |
| Христина Мисак | Створення презентації |


### Відгуки команди:

#### Артем Краєвський

По-перше, це був дуже цікавий та веселий досвід. По-друге, наглядний приклад того, як ті дисципліни, що ми вивчаємо, разом можуть знадобитися у подальшому житті.

#### Анастасія Диня

Це був цікавий та неймовірно важливий досвід, який показав, що дискретна математика має дуже цікаве і практичне застосування у реальному житті

#### Софія Шуляк

Цей досвід я вважаю цінним, адже ми розглядали прикладне застосування дискретної математики. Також цей проєкт був поштовхом для мене до подальшого ознайомлення з GitHub, що є не менш важливим.

#### Христина Мисак

Робота у команді над проєктом була цікавою та налагодженою, і водночас показала, що теорія графів має дуже широке застосування у багатьох сферах. Особливо сподобався алгоритм на проміжному етапі, який займав приблизно 4 години (хоча це, мабуть, ще не найдовше).

#### Михайло Еліосов

Дуже сподобалося працювати над optimization problem, а в процесі пошуків алгоритмів я прочитав багато real world use cases таких алгоритмів, завдяки чому нарешті зрозумів, навіщо ми вивчаємо дискретну математику
