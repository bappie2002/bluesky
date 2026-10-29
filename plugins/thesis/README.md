# Thesis-plugins voor BlueSky

Vier plugins voor de aankomstsimulatie bij de droneport. BlueSky vindt ze vanzelf
in deze map (`bluesky/plugins/thesis/`); je hoeft niets in te stellen.

| Plugin | Bestand | Wat hij doet |
|---|---|---|
| ENERGY | `energy.py` | Houdt per drone de accu bij: elke seconde energie = vermogen × tijd, ook per vliegmodus. |
| FLIGHTLOG | `flightlog.py` | Schrijft drie CSV-bestanden per run: per drone, per gebeurtenis en (optioneel) per seconde. |
| VPMS | `vpms.py` | De structuur van Chang et al. (2026), geschaald: gates, bogen, merge point, laatste daling, platform. Met FCFS (case C1). |
| TRAFFIC | `thesis_traffic.py` | Het verkeer en de verstoringen van één run. |

(ARRIVALS is op 27-10 vervangen door TRAFFIC en verwijderd.)

## Snel proberen

**In de GUI:** start BlueSky en typ `IC thesis/demo_vpms.scn`.
60 drones per uur, 10 minuten gemeten, voorspellingsfout 10%, gedeelde bogen. Geen wind.

**Zonder GUI (tests):** vanuit de map `bluesky`:

```
python plugins/thesis/tests/test_energy.py
python plugins/thesis/tests/test_flightlog.py
python plugins/thesis/tests/test_vpms.py shared 0 60     # variant, voorspellingsfout (%), drones/uur
python plugins/thesis/tests/test_together.py
```

Elke test print de uitkomst naast wat je mag verwachten. Draai na elke
wijziging alle vier opnieuw.

## De plugins

### ENERGY
- Vermogen per drone uit zijn configuratie en luchtsnelheid:
  - multicopter: één curve van hover tot maximale snelheid;
  - Lift+Cruise: boven de overtreksnelheid op de vleugel, eronder op de rotors.
- Transities van de Lift+Cruise (Mathur, Table 9, geschaald met het hoververmogen), als blok
  dat in die tijd het vermogen van de curve vervangt:
  - vleugel → rotors: start als hij onder de overtreksnelheid zakt; 10 s, 3,9 Wh;
  - rotors → vleugel: start als hij onder de overtreksnelheid de opdracht krijgt om op of boven
    de overtreksnelheid te vliegen; 15 s, 4,5 Wh (sinds 27-10).
- Welke drone welke configuratie is, volgt uit het BlueSky-type: `AMZN` = Lift+Cruise,
  al het andere (bijv. `M200`) = multicopter. Het type bepaalt alleen hoe de drone
  beweegt; het energieverbruik komt uit deze plugin.
- **Het energiemodel staat niet in dit bestand.** Het heeft één plek, in de map `experimenten`:
  formules in `experimenten/plugins/energy_model.py`, waarden in `experimenten/config/vehicles.py`.
  `energy.py` importeert die en past ze elke seconde toe op elke drone.
  De map `experimenten` moet naast `bluesky` staan (in `afstuderen/`), of zet de
  omgevingsvariabele `EXPERIMENTEN` op het pad (bijv. op een cluster).
  Onderbouwing: `experimenten/config/aannames.md` en `Energiemodel_uitleg.docx`.
- Houdt per drone ook bij: vermogen nu (W), vliegmodus en energie per modus.
  Modi: `hover` (luchtsnelheid onder 0,5 m/s), `rotor` (vooruit op de rotors: multicopter, of L+C
  onder overtreksnelheid), `wing` (L+C op de vleugel). Transities (beide richtingen) apart in
  `wh_trans` en `n_trans`.
- Commando: `SOC MC1` toont de accu van MC1, `SOC MC1 0.6` zet hem op 60%.

### FLIGHTLOG
- Commando: `FLIGHTLOG naam` schrijft naar de map `output/naam/`:
  - `flights.csv`: één regel per drone, als hij na de landing van het platform gaat.
    Tijden, SoC begin en eind, energie totaal en per modus (`wh_hover`, `wh_rotor`,
    `wh_wing`, `wh_trans`), tijd per modus, aantal transities, kleinste afstand, tijd in LoS.
  - `events.csv`: één regel per gebeurtenis: `spawn`, wissel van modus (event = nieuwe
    modus, info = oude), `los_start`/`los_end` (info = de andere drone), de events van VPMS
    en TRAFFIC (hieronder), `end`. Met de toestand op dat moment.
- `FLIGHTLOG naam TRACE` schrijft daarnaast `trace.csv`: elke drone elke seconde
  (positie, snelheid, modus, vermogen, energie, SoC). Groot; alleen voor figuren en debuggen.
- Verlies van separatie: dichter dan 32 m horizontaal én 7,62 m verticaal (Metropolis II, besluit 23-10).
- Figuren en samenvatting: `python analysis/run_report.py ../bluesky/output/naam`
  (vanuit de map `experimenten`).

### VPMS
- Pad op 52,0 N 4,0 O; merge point 40 m erboven. Vier gates op de hoogte van hun boog (100 of 90 m), 480 m van het pad (buitenste boog + 100 m, `GATE_MARGIN`); de daling vanuit het transit-luchtruim gebeurt vóór de gate (besluit 29-10). Minimaal `T_GATE` = 10 s tussen twee drones bij dezelfde gate (het kinematische minimum van 9,7 s, naar boven afgerond) (`VPMS GATE s`).
- Per gate een boog van 150° met de klok mee, op 100 m (gates 0 en 2) of 90 m (gates 1 en 3).
- Straal = 2 × 40 s × wachtsnelheid / 150°: `VPMS SHARED` één boog per gate (380 m, L+C bepaalt),
  `VPMS SPEED` per gate een MC-boog (174 m) en een L+C-boog (380 m). `VPMS SHOW` tekent ze.
  `VPMS TARGET 35` zet het doel op 35 s (de straal volgt mee); vóór het verkeer.
- Per drone: entry → boog → (hover aan het eind) → direct naar het merge point (komt in hover aan)
  → verticaal dalen (2 m/s tot 5 m, dan 0,5 m/s) → platform (2 s + tijd om hem eraf te halen) → weg.
- Boog volgen: de drone mikt op een punt een vooruitkijkafstand verder op de boog (2 × draaicirkel),
  en begint zijn bocht de boog op op die afstand. Zo schiet hij niet door.
- Niet inhalen, vanaf de gate (besluit 29-10): ingangsstuk en boog vormen één rijstrook per boog. Een volger
  houdt minstens 64 m (`GAP_MIN` = 2 × 32 m) tot zijn voorganger; de 40 s is een wens bij het merge point,
  geen regel in de rijstrook (`ARC_SPACING`, `VPMS DISTANCE`; `VPMS TIME` geeft de oude regel voor de
  vergelijking). Een drone die de boog verlaat blijft voorganger tot hij 37 m binnen de boog is, en een
  volger kan altijd 64 m vóór het eind van de boog stoppen. L+C achter een MC: van de vleugel af.
- Geen wachten vóór de gate (optie B, besluit 29-10): drones komen op hun getrokken tijd binnen; drukte
  wordt in de structuur opgevangen.
- Harde minima: nooit binnen 7,62 m boven de drone eronder (ook boven een drone op het platform);
  het merge point is pas vrij als de drone eronder 7,62 m lager is; anders wacht de volgende op 35 m.
- FCFS: vrijgegeven in volgorde van binnenkomst op de boog, ten minste 40 s na de vorige bij het merge
  point. Een latere SEQUENCER-plugin vervangt alleen `next_clearance()`.
- Verstoring: de sequencer plant met een voorspelde vliegtijd naar het merge point met een fout per drone
  (van TRAFFIC); de drone vliegt de echte tijd. Daardoor wijken de echte intervallen af van de geplande.
- Op het direct-been houdt een drone zijn boog-hoogte tot 37 m binnen de boog, zodat hij over de
  lagere buurboog gaat met de volle 10 m ertussen.
- Wind wordt in dit onderzoek niet gesimuleerd (besluit 27-10). VPMS kan het wel (hoveren = neus in de
  wind, positie vastgehouden), voor later onderzoek.
- Zet BlueSky's CAS/Mach-grens op 0,01 m/s (anders leest BlueSky 0,1-2 m/s als Mach).
- Events: `arc_in`, `hold`, `direct_to`, `merge`, `touchdown`.

### TRAFFIC
- `TRAFFIC vraag seed [fout_%] [BASE|LONG] [inloop_s] [meet_s]`; standaard 3600 s inloop + 3600 s meten.
- Poisson-aankomsten, gate uniform, 50/50 types, SoC uniform 40-80%, platformtijd 2 s + U(2, 6) s (LONG: 60 s).
- Voorspellingsfout per drone: normaal verdeeld, standaardafwijking fout_% (niveaus 0, 5, 10, 20%; eigen aanname).
  Hoofdvergelijking zonder fout; separatie-analyse met alle niveaus.
- Eigen random-stroom per grootheid uit één seed: alle varianten krijgen dezelfde drones.
- Wacht een drone voor de gate (boog nog niet vrij), dan kost dat energie op zijn wachtvermogen.
- Stopt (HOLD) als alle drones uit het meetvenster geland zijn. Events: `measured` of `runin`.
- `TRAFFIC BATCH` schrijft de hoofdvergelijking: `scenario/thesis/batch_shared.scn` en `batch_speed.scn`
  (5 drukteniveaus × 100 seeds = 500 runs per variant).
- `TRAFFIC BATCH SEP` schrijft de separatie-analyse: `batch_sep_shared.scn` en `batch_sep_speed.scn`
  (5 doelen × 4 foutniveaus × 5 drukteniveaus × 100 seeds = 10.000 runs per variant).
- Beide nog niet getest in BlueSky's batch-modus.

## Nog open
- Afstand gate tot buitenste boog (`GATE_MARGIN` = 100 m, eigen keuze), MC-snelheid op het entry-been (nu 9,0 m/s).
- Waar drones wachten buiten de structuur.
- De L+C stopt in 83 m (Mathur) via een vaste vertraging.

## Goed om te weten
- Tijdstap: in de tests staat `DT 0.5` (standaard 0,05 s). Dat is 10× sneller.
- Een volledige run van 2 uur kost ongeveer 15 s rekentijd.
- Een parameter van het energiemodel verander je alleen in `experimenten/config/vehicles.py`;
  de volgende simulatie gebruikt hem meteen. Draai daarna `check_excel.py` als je de Excel
  gelijk wilt houden.

## Volgende plugin
SEQUENCER: de energiebewuste regels (vervangt `next_clearance()` in VPMS).
