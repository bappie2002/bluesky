# Thesis-plugins voor BlueSky

Vier plugins voor de aankomstsimulatie bij de droneport. BlueSky vindt ze vanzelf in deze map. Hoe ze samen werken, met elk
getal: `experimenten/docs/UITLEG.md`. `archief/` bevat alleen documenten (nooit een `.py` daar neerzetten: BlueSky laadt
elk `.py`-bestand onder `plugins/`). Na een wijziging: `python experimenten/tools/controle.py` vanuit `afstuderen`.

| Plugin | Bestand | Wat hij doet |
|---|---|---|
| VPMS | `vpms.py` | De structuur (Chang et al. 2026, geschaald) en haar regels, met FCFS-vrijgave (C1, C2) |
| TRAFFIC | `thesis_traffic.py` | Het verkeer van één run: aankomsten per gate, types, SoC, padtijd, voorspelfout |
| ENERGY | `energy.py` | De accu per drone: elke seconde vermogen × tijd, per vliegmodus, transities als blok |
| FLIGHTLOG | `flightlog.py` | Logbestanden per run: `flights.csv`, `events.csv`, `run.json` (en `trace.csv` met TRACE) |

Elk bestand begint met een korte uitleg van wat het doet. Waarom elke keuze zo is, en wat ze verving:
`experimenten/docs/BESLUITEN.md`.

## Waar staan de getallen?

**Alles in één bestand: `experimenten/config/model.py`**, in zeven delen: energiemodel, voertuigen, structuur en regels,
verkeer en logs, geometrie, vooruit vliegen (aankomst en daalplan), lane-regel en aankomsten. De plugins importeren het
als `M` en doen alleen het BlueSky-werk: drones sturen, tellen en loggen. Commando's als `VPMS SPEED` veranderen de
instelling in `model.py` zelf (`M.VARIANT`), dus lees instellingen altijd als `M.NAAM`.

Alleen de grenzen van de BlueSky-types GMC en GLC (max snelheid, versnelling, plafond) staan in BlueSky zelf:
`bluesky/resources/performance/OpenAP/rotor/aircraft.json`. `test_vpms.py` controleert of ze kloppen met `model.py`.

De map `experimenten` moet naast `bluesky` staan (in `afstuderen/`), of zet de omgevingsvariabele `EXPERIMENTEN`.

## Commando's

```
VPMS SHOW | SHARED | SPEED | TARGET s | OFFSET m | LEG m | RING ON/OFF | RESTART ON/OFF | HORIZON s
(LEG zet ook de horizon die erbij hoort; geef HORIZON daarna. RESTART: G2, een hoverende L+C vliegt alsnog door)
TRAFFIC vraag seed [fout_%] [BASE|LONG] [warmup_s] [meet_s]        TRAFFIC VIDEO ON|OFF
FLIGHTLOG naam [TRACE]
SOC drone [waarde]
```

## Tests (vanuit de map `bluesky`)

```
python plugins/thesis/tests/test_energy.py
python plugins/thesis/tests/test_flightlog.py
python plugins/thesis/tests/test_together.py
python plugins/thesis/tests/test_vpms.py shared 0 60                    # variant, fout %, vraag/h
python plugins/thesis/tests/test_vpms.py speed 0 90 40 300 1800 OFF     # + target, warm-up, meten, ring
python plugins/thesis/tests/test_vpms.py speed 0 90 40 300 1800 ON ON   # + herstart vanuit hover (G2), standaard OFF
```

Elke test print de uitkomst naast wat je mag verwachten. De ontwerpchecks van de hele structuur:
`experimenten/ontwerpchecks/` (`designchecks.py`).

## In de GUI

`IC thesis/kijk_c1.scn` of `kijk_c2.scn`: één run uit de batch (60/h, seed 1). `IC thesis/demo_vpms.scn`: korte demo.
Niet doen tijdens een run: SPD/ALT/HDG/DEL op drones of een andere DT dan 0.5; VPMS stuurt zelf.
