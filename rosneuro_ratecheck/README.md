TEST RATE:

rosrun rosneuro_ratecheck test_rate _sampling_freq:=590 _sub_topic_data:=/neurodata

TEST DELAY: 

rosrun rosneuro_ratecheck test_delay _expected_delay:=0.0 _first_sub_topic:=/neurodata _second_sub_topic:=/neuroprediction


Per Luca:
Il primo nodo (ratecheck) controlla su singolo topic la frequenza di pubblicazione e manda warning se è troppo diversa da quella prevista. Al momento il codice manda il warning ogni iterazione, dimmi se sarebbe meglio far gestire la situazione in un altro modo (tipo mandando solo un warning e facendo solo un controllo periodico).

Il secondo nodo salva in una coda i messaggi ricevuti dal primo topic (così da non perderne nessuno nel caso in cui il delay fra i due topic fosse molto alto) e ad ogni arrivo di un messaggio dal secondo topic toglie dalla coda il messaggio con lo stesso numero di sequenza e calcola il delay usando gli stamp dell'header.
Il codice è un po' sporco, ho dovuto aggiungere dei metodi per guardare il primo elemento della cosa senza estrarlo perchè mi stavo trovando con messaggi del secondo topic che avevano un seq minore di quelli del primo topic (per via del problema dei numeri sequenza di cui ti ho parlato e che abbiamo risolto con neuroheader), quindi in quel caso non toglievo elementi dalla coda e aspettavo arrivasse un messaggio dal secondo topic con numero di sequenza pi
 alto per cominciare a calcolare il delay.
 
 
