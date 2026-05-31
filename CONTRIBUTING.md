# Guida alla navigazione della repository

Questo file è utilizzato come guida per il docente, per rendere piu' semplice
capire cosa è stato aggiunto dal gruppo, quali file sono stati modificati
rispetto alla repository iniziale e dove cercare le parti principali del
lavoro.

## Struttura della repository

Sono stati aggiunti o modificati i seguenti file:

- `input_shaper_filter.py`

  Contiene l'implementazione dei filtri di input shaping richiesti
  dall'assignment:

  - `ZV`, Zero Vibration;
  - `ZVD`, Zero Vibration and Derivative;
  - `ZVDD`, Zero Vibration and Double Derivative;
  - `EI`, Extra Insensitive.

  Il file contiene una classe `InputShaperFilter`, pensata per essere
  riutilizzabile direttamente nello script di simulazione. Il tipo di filtro
  viene scelto tramite l'enum `ShaperType`, ad esempio:

  ```python
  input_shaper = InputShaperFilter(
      Tc=Tc,
      filter_type=ShaperType.ZVD,
      initial_reference=initial_reference
  )
  ```

  I parametri comuni del sistema sono definiti come attributi della classe:

  - lunghezza equivalente del pendolo `L_PENDULUM`;
  - gravità `GRAVITY`;
  - coefficiente di smorzamento stimato `XI`;
  - frequenza naturale `omega_n`;
  - frequenza smorzata `omega_d`;
  - periodo oscillatorio `T`;
  - decay ratio `K`, usato nel calcolo delle ampiezze dei filtri smorzati.

  Ogni metodo `ZV`, `ZVD`, `ZVDD` ed `EI` calcola le ampiezze e i ritardi del
  filtro corrispondente. Questi valori vengono poi passati a `create_filter`,
  che converte i ritardi da secondi a campioni e inizializza la memoria interna
  del filtro.

  Il metodo `filter(reference)` è quello usato durante la simulazione. Riceve
  la reference corrente `[q, dq, ddq]` e restituisce la reference filtrata,
  ottenuta come somma pesata di campioni correnti e ritardati. La memoria serve
  proprio a conservare i valori passati della reference, necessari per applicare
  i ritardi degli impulsi.

- `impulse_response.py`

  Script aggiunto per stimare sperimentalmente lo smorzamento del modo
  pendolare. Lo script è volutamente simile alla simulazione principale:
  carica lo stesso modello MuJoCo del carroponte, inizializza il robot e usa lo
  stesso passo di campionamento.

  Invece di usare la traiettoria `test_trj1.txt`, viene applicato un impulso
  approssimato tramite un breve impulso rettangolare di area unitaria. Dopo
  l'impulso la forza viene riportata a zero, in modo da osservare la risposta
  libera del pendolo.

  Lo script salva nel tempo l'angolo del pendolo `theta`, lo mostra in un
  grafico Plotly e calcola una stima di `XI` tramite decremento logaritmico. In
  particolare vengono cercati i picchi positivi della risposta e viene calcolato
  lo smorzamento confrontando il primo picco con i successivi. Il valore medio
  ottenuto è stato poi usato in `input_shaper_filter.py`.

- `robot_simulation.py`

  Questo è lo script principale usato per confrontare i diversi filtri durante
  la simulazione del carroponte.

  Le modifiche principali sono:

  - import della classe e dell'enum:

    ```python
    from input_shaper_filter import InputShaperFilter, ShaperType
    ```

  - creazione del filtro di input shaping:

    ```python
    input_shaper = InputShaperFilter(
        Tc=Tc,
        filter_type=ShaperType.EI,
        initial_reference=initial_reference
    )
    ```

    Per cambiare filtro è sufficiente sostituire `ShaperType.EI` con
    `ShaperType.ZV`, `ShaperType.ZVD` oppure `ShaperType.ZVDD`.

  - applicazione del filtro dentro il ciclo di simulazione:

    ```python
    reference = input_shaper.filter(reference)
    ```

    Da questo punto in poi il controllore riceve direttamente la reference
    filtrata. Il controllore già presente nella repository non è stato
    modificato.

## Dati generati

Durante l'esecuzione di `robot_simulation.py`, i risultati delle simulazioni
vengono salvati in:

```text
crane/tests/
```

La cartella `tests` è stata aggiunta al `.gitignore` per evitare di appesantire la repository, ma i dati sono comunque disponibili localmente e possono essere rigenerati eseguendo lo script.

## Conclusioni

In conclusione, la repository originale è stata mantenuta il piu' possibile
inalterata nella parte di controllo. Il contributo principale è stato
l'inserimento di uno stadio di input shaping prima del controllore, in modo da
modificare la reference del carrello senza cambiare la logica di controllo già
presente.

Sono stati quindi aggiunti:

- una classe riutilizzabile per i filtri ZV, ZVD, ZVDD ed EI;
- uno script per identificare sperimentalmente lo smorzamento del pendolo;
- poche modifiche mirate allo script principale di simulazione per applicare e
  confrontare i filtri.

### Autori

- Stefano Agnelli
- Antonio Di Filippo
- Wen Wen Sun
