# Charakterystyka tematu projektu WizualizacjaSM
 
Głównym celem projektu jest stworzenie robota Micromouse który będzie posiadał tylko akcelerometr i żyroskop do rozpoznawania położenia i przeszkód. Posiadać on będzie pierścień zamontowany na sprężynie dająca niewielkie możliwości ruchowe, w momencie uderzenia ze ścianą pierścień odchyli się a czujnik to wykryje. Robot jest też projektem na przedmiot Roboty Mobilne. Aby komunikować się z komputerem potrzebne jest urządzenie pośredniczące które będzie również wyposażone w moduł radiowy i interfejs usb aby można było dane przesyłać do komputera. Urządzenie pośredniczące to projekt na Sterowniki Robotów, jest to joystick który posiadać będzie interfejs USB, Wifi, Bluetooth i moduł radiowy, będzie więc idealnym łącznikiem Robot - Komputer.

# Algorytm Micromouse
Algorytm ma za zadanie wykrywać ściany w labiryncie i zapamiętywać ich ustawienie w pamięci, po odkryciu całego labiryntu wyznaczana będzie ścieżka optymalna z punktu startowego do centrum labiryntu, czyli mety. Następnie robot będzie musiał przebyć wyznaczoną ścieżkę jak najszybciej. Algorytm został zaimplementowany w aplikacji. Takie rozwiązanie bardzo usprawniło tworzenie algorytmu. Istotne jest to że wymiana danych mimo swej prędkości nie jest wstanie zapewnić dokładności w takich operacjach jak przemieszczanie się czy obracanie, liczenie podwójnej całki z tak otrzymywanych danych byłoby bezużyteczne, robot więc posiada funkcje do przemieszczania się i obrotu samodzielnie, zapewnia to dokładność. Jazda o zadane przemieszczenie odbywa się poprzez dostarczenie też informacje o wartości progowej przyspieszenia które oznaczałoby zderzenie się z przeszkodą, niezależnie od przyczyny zakończenia ruchu, robot wysyła wartość przemieszczenia jaką udało mu się wyliczyć na podstawie odczytów z akcelerometru. Podczas obrotu nie podaje się warunku stopu. Uzyskana dokładność obrotu jest bardzo zadowalająca, lecz przemieszczanie się jest obarczone takim błędem że nie możliwe jest zadanie robotowi przemieszczenia większego niż 18 cm (rozmiar jednej komórki labiryntu) bez błędu na tyle dużego który zakłóci działanie algorytmu. Problem ten rozwiązuje sposób przemieszczania się w którym robot zawsze przemieszcza się najdalej jak potrafi, algorytm dostając informację zwrotną dopasowywać ją do labiryntu, jednak takie rozwiązanie powoduje następny problem, robot będzie mógł wpaść w pułapki z których nie będzie dało się w ten sposób wyjechać, rozwiązaniem jest połączenie tych dwóch taktyk tak by algorytm sam decydował o sposobie przemieszczania się.


![GitHub Logo](/images/a.jpg)

# Linki

https://youtu.be/gzY4Sm2h51A - Prezentacja ukończenia projektu

https://www.overleaf.com/read/rbngfpkfhwbz - Raport końcowy
