# Thesis-plugins voor BlueSky

Drie plugins voor de aankomstsimulatie bij de droneport. BlueSky vindt ze vanzelf
in deze map (`bluesky/plugins/thesis/`); je hoeft niets in te stellen.

| Plugin | Bestand | Wat hij doet |
|---|---|---|
| ENERGY | `energy.py` | Houdt per drone de accu bij: elke seconde energie = vermogen × tijd, ook per vliegmodus. |
| FLIGHTLOG | `flightlog.py` | Schrijft drie CSV-bestanden per run: per drone, per gebeurtenis en (optioneel) per seconde. |
| ARRIVALS | `arrivals.py` | Maakt synthetisch aankomstverkeer aan bij de gates. |

## Snel proberen

**In de GUI:** start BlueSky en typ `IC thesis/demo_arrivals.scn`.
Er komen 60 drones per uur binnen, 10 minuten lang. Ze vliegen nu nog recht
naar het pad (tijdelijk, tot de VPMS-plugin er is).

**Zonder GUI (tests):** vanuit de map `bluesky`:

```
python plugins/thesis/tests/test_energy.py
python plugins/thesis/tests/test_flightlog.py
python plugins/thesis/tests/test_arrivals.py
python plugins/thesis/tests/test_together.py
```

Elke test print de uitkomst naast wat je mag verwachten. Draai na elke
wijziging alle vier opnieuw.

## De plugins

### ENERGY
- Vermogen per drone uit zijn configuratie en luchtsnelheid:
  - multicopter: één curve van hover tot maximale snelheid;
  - Lift+Cruise: boven de overtreksnelheid op de vleugel, eronder op de rotors.
- Zakt een Lift+Cruise onder de overtreksnelheid, dan kost de transitie één keer extra energie.
- Welke drone welke configuratie is, volgt uit het BlueSky-type: `AMZN` = Lift+Cruise,
  al het andere (bijv. `M200`) = multicopter. Het type bepaalt alleen hoe de drone
  beweegt; het energieverbruik komt uit deze plugin.
- Parameters staan bovenaan het bestand, de formules (1) t/m (9) eronder met uitleg.
  Onderbouwing: `experimenten/config/aannames.md` en `Energiemodel_uitleg.docx`.
- Houdt per drone ook bij: vermogen nu (W), vliegmodus en energie per modus.
  Modi: `hover` (onder 0,5 m/s), `rotor` (vooruit op de rotors: multicopter, of L+C onder
  overtreksnelheid), `wing` (L+C op de vleugel). Transities apart in `wh_trans`.
- Commando: `SOC MC1` toont de accu van MC1, `SOC MC1 0.6` zet hem op 60%.

### FLIGHTLOG
- Commando: `FLIGHTLOG naam` schrijft naar de map `output/naam/`:
  - `flights.csv`: één regel per drone, bij verwijderen (straks: na de landing).
    Tijden, SoC begin en eind, energie totaal en per modus (`wh_hover`, `wh_rotor`,
    `wh_wing`, `wh_trans`), tijd per modus, aantal transities, kleinste afstand, tijd in LoS.
  - `events.csv`: één regel per gebeurtenis: `spawn`, wissel van modus (event = nieuwe
    modus, info = oude), `los_start`/`los_end` (info = de andere drone), `end`. Met de toestand
    op dat moment. Hieruit is de tijdlijn van elke drone terug te bouwen.
- `FLIGHTLOG naam TRACE` schrijft daarnaast `trace.csv`: elke drone elke seconde
  (positie, snelheid, modus, vermogen, energie, SoC). Groot; alleen voor figuren en debuggen.
- Andere plugins kunnen eigen events toevoegen (straks VPMS: `arc_in`, `direct_to`, `touchdown`).
- Delay komt erbij zodra de VPMS-plugin de vroegst mogelijke landingstijd geeft.
- Figuren en samenvatting: `python analysis/run_report.py ../bluesky/output/naam`
  (vanuit de map `experimenten`).

### ARRIVALS
- Trekt vooraf de complete aankomstlijst uit één seed: tijd, gate, type en SoC.
  **Dezelfde seed geeft precies hetzelfde verkeer**, dus alle cases krijgen dezelfde samples.
- Aankomsten vormen een Poisson-proces (exponentiële tijd tussen drones).
- Commando: `ARRIVALS 120 3600 1` = 120 drones per uur, 3600 s lang, seed 1.

## Nog te beslissen (placeholders bovenaan de bestanden)
- `flightlog.py`: `SEP_H`, `SEP_V` (separatieminima).
- `arrivals.py`: `GATE_DIST`, `ALT`, `GATE_BEARINGS` (volgen uit het VPMS-ontwerp),
  `SPEED` per type, `SOC_RANGE` (ondergrens boven reserve + maximale absorptie).

## Goed om te weten
- Tijdstap: in de tests staat `DT 0.5` (standaard 0,05 s). Dat is 10× sneller.
- BlueSky start niet zonder verkeer; `ARRIVALS` stuurt daarom zelf `OP`.
- In `experimenten/plugins/energy.py` staat hetzelfde energiemodel, voor de
  Excel en de analyses. Verander je een parameter, doe dat dan op beide plekken.

## Volgende plugin
VPMS: gates, arcs, merge point en pad als routes, plus het landen (drone verwijderen
na touchdown). Daarna FCFS (case C1).
