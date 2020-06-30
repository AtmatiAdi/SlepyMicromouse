# OpisProjektu

Głównym celem projektu jest stworzenie Micromouse który będzie posiadał tylko akcelerometr i żyroskop do rozpoznawania położenia przeszkód i położenia samego robota. Takie rozwiązanie jest niekonwencjonalne gdyż z reguły wszystkie roboty tego typu posiadają czujniki optyczne do wykrywania przeszkód. Micromouse to kategoria robotów których zadaniem jest wydostanie się z labiryntu, ten robot będzie miał takie samo zadanie lecz strategia wyszukiwania wyjścia z labiryntu będzie inna niż standardowo. Robot aby zidentyfikować ściany które są dookoła niego musi w nie stuknąć, nie za mocno by nie uszkodzić labiryntu, a zamantowany akcelerometr wykryje kierunek uderzenia i tak będzie wiadomo gdzie znajduje się przeszkoda. Wykrycie kierunku uderzenia nie jest trywialne więc do tego posłuży specjalnie zamontowana obręcz na sprężynie która da niewielkie możliwości ruchowe, tak aby przy uderzeniu obrącz lekko się przesunęła i akcelerometr na niej zamontowany mógł poprawnie odczytać przesunięcie. Ruchoma obręcz jest potrzebna gdyż zakładamy że koła nie mają poślizgu, więc wykrycie kierunku byłoby utrudnione. żyroskop daje możliwość odczytu obrotu całego robota, wszystko to jest potrzebne ponieważ robot nie będzie miał enkoderów ani innych sensorów które pozwolą sledzić jego przemieszczenie.

![GitHub Logo](/images/a.jpg)

Robot posiada mikrokontroler STM32F103 i sensor MPU-6050. Silniki dobrane będą tak by robot dostatecznie precyzyjnie mógł się poruszać, koła dobrane są tak aby zapewnić przyczepność i móc między innymi w algorytmie wykluczyć poślizg. Dodatkowo wykorzystany zostanie moduł nRF24L01 do komunikacji zdalnej, aby robotem sterować i pobierać od niego informacje sensoryczne, będzie to część na projekt wizualizacja danych sensorycznych. Dodatkowo pomiar napięcia baterii by zabezpieczyć baterie przed rozładowaniem. Moduł mikrokontrolera zawiera stabilizator, tak jak IMU czy moduł radiowy, więc na schemacie zostatał on pominięty.

Sensor MPU-6050 Jest 3 osiowym akcelerometrem i 3 osiowym żyroskopem w jednym, komunikacja z nim przebiega przez magistralę I2C
moduł nRF24L01 To moduł radiowy 2.4Ghz, komunikacja z nim odbywa się poprzez magistralę SPI
Schemat modułowy robota przedstawia rysunek:

![GitHub Logo](/images/Modules.PNG)

# Algorytm Micromouse
Algorytm ma za zadanie wykrywać ściany w labiryncie i zapamiętywać ich ustawienie w pamięci, po odkryciu całego labiryntu wyznaczana będzie ścieżka optymalna z punktu startowego do centrum labiryntu, czyli mety. Następnie robot będzie musiał przebyć wyznaczoną ścieżkę jak najszybciej. Algorytm został zaimplementowany w aplikacji. Takie rozwiązanie bardzo usprawniło tworzenie algorytmu. Istotne jest to że wymiana danych mimo swej prędkości nie jest wstanie zapewnić dokładności w takich operacjach jak przemieszczanie się czy obracanie, liczenie podwójnej całki z tak otrzymywanych danych byłoby bezużyteczne, robot więc posiada funkcje do przemieszczania się i obrotu samodzielnie, zapewnia to dokładność. Jazda o zadane przemieszczenie odbywa się poprzez dostarczenie też informacje o wartości progowej przyspieszenia które oznaczałoby zderzenie się z przeszkodą, niezależnie od przyczyny zakończenia ruchu, robot wysyła wartość przemieszczenia jaką udało mu się wyliczyć na podstawie odczytów z akcelerometru. Podczas obrotu nie podaje się warunku stopu. Uzyskana dokładność obrotu jest bardzo zadowalająca, lecz przemieszczanie się jest obarczone takim błędem że nie możliwe jest zadanie robotowi przemieszczenia większego niż 18 cm (rozmiar jednej komórki labiryntu) bez błędu na tyle dużego który zakłóci działanie algorytmu. Problem ten rozwiązuje sposób przemieszczania się w którym robot zawsze przemieszcza się najdalej jak potrafi, algorytm dostając informację zwrotną dopasowywać ją do labiryntu, jednak takie rozwiązanie powoduje następny problem, robot będzie mógł wpaść w pułapki z których nie będzie dało się w ten sposób wyjechać, rozwiązaniem jest połączenie tych dwóch taktyk tak by algorytm sam decydował o sposobie przemieszczania się.




# Linki

https://youtu.be/gzY4Sm2h51A - Prezentacja ukończenia projektu

https://www.overleaf.com/read/rbngfpkfhwbz - Raport końcowy
