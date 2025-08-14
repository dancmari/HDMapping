# Lidar Odometry – procesare flexibilă cu Stop / Rollback / Modificare parametri / Resume (plan)

Acest document propune o strategie și un plan incremental pentru a permite oprirea procesării în orice moment, derularea înapoi (rollback) la un punct sigur, modificarea parametrilor (preferabil din GUI) și reluarea procesării cu noile setări. Scopul este obținerea unor rezultate corecte în situații atipice (accelerații mari, schimbări bruște de viteză/mediu, staționare), fără a re-rula întreaga sesiune.

## Considerații / observații (pe scurt)
Scop: un rezumat rapid care ajută la înțelegerea „cum funcționează acum” și „ce putem face” imediat și ulterior.

- Cum funcționează acum (stare actuală Pasul 1):
  - Odometrie locală: IMU (AHRS) ca seed de orientare, apoi NDT/SF/rigid_sf iterative cu early‑stop; fără loop closures în Pasul 1.
  - Procesare pe chunk‑uri de `threshold_nr_poses`, hartă rulantă (RGD), filtrare/decimare ușoară; scriere progresivă per‑chunk (LAZ + trajectory_lio_chunk_i.csv) și artefacte de sesiune la final.
  - Parametrii rămân constanți pe durata rularii; `real_time_threshold_seconds` limitează doar bucla LO principală (nu întrerupe sub‑pașii NDT).
  - Resume/rollback: nu există încă `state.json` sau writer atomic `.part`; reluarea/crash‑safe nu este garantată automat (vezi secțiunile „Stop/Resume” și „Persistență sigură”).

- Limitări actuale (de adresat în acest plan):
  - Lipsesc `state.json`, writer atomic și versionarea parametrilor per‑chunk (`params_timeline.json`).
  - Risc de supra‑potrivire pe obiecte dinamice (nu excludem explicit ultimele M poze din ținta NDT).
  - Telemetrie/indicatori de încredere agregați sunt minimali; deciziile de rafinare sunt manuale.
  - Exporturi extinse (traiectorie îmbogățită, calitate, nor asamblat) și re‑procesare de segment nu sunt încă implementate.

- Ce putem face imediat (MVP cu risc scăzut):
  - Introducem `state.json` + scriere atomică `.part` + curățare/scanare la start pentru Resume crash‑safe; butoane GUI Stop/Resume.
  - Adăugăm `ndt_exclude_recent_poses` (1–2 în scene dinamice) pentru defazajul țintei și mai multă robustețe.
  - Profiluri și rafinare selectivă bugetată: când `confidence_score` e sub prag, aplicăm +1 iterație și/sau `epsilon *= 0.5` în limita `max_refinement_time_ms_per_chunk`.
  - Telemetrie ieftină per‑chunk: iters, stop_reason (converged|plateau|time_budget), scor NDT/inlier_ratio, `confidence_score` – scrise în CSV/JSON de calitate.
  - Traiectorie îmbogățită (de bază): CSV separat cu câțiva scalari utili (match_score, inlier_ratio, iters, confidence_score) la o rată moderată (ex. 10 Hz).

- Ce putem face ulterior (extins):
  - Snapshots RGD pentru resume rapid; auto‑parametrizare (assist/offline/online) cu gardă de timp; detectare/etichetare obiecte în mișcare cu bugete.
  - Exporturi suplimentare (calitate detaliată, nori asamblați pe segmente), re‑procesare de segment cu „splice” sigur, UX GUI mai bogat și politici de compatibilitate parametri.

- Recomandări rapide (până la integrarea completă):
  - Scene dinamice (urban/traﬁc): `profile="balanced"`, `ndt_exclude_recent_poses = 2`, `max_refinement_time_ms_per_chunk ≈ 50` ms; exporturi extinse OFF.
  - Scene relativ statice (pădure/interior stabil): `profile="fast"`, defazaj OFF, doar export traiectorie simplă.
  - Dacă apar staționări lungi: activați pre‑pass IMU + ZUPT (când disponibil) pentru a reduce iterațiile în ferestrele respective.

Trimiteri: vezi secțiunile „Mitigare obiecte dinamice”, „Viteză vs. Calitate”, „Automatizare (headless/batch)”, „Exporturi” și „Reprocesare segment” pentru detalii și parametri concreți.

## Obiective (MVP → extins)
- MVP
  - Stop sigur între chunk-uri (nu în mijlocul unui sub-pas).
  - Resume crash-safe și voluntar (după Stop) de la următorul chunk neprocesat.
  - Rollback la ultimul „checkpoint” confirmat și re-procesare de acolo.
  - Param set versioning: fiecare chunk știe cu ce parametri a fost procesat.
  - Scriere imediată a rezultatului pe chunk, atomica și idempotentă (evităm corupția).
- Extins
  - Rollback cu fereastră parametrizabilă (N chunk-uri / T secunde / N poze).
  - Snapshots RGD (hartă rulantă) la fiecare K chunk-uri pentru resume rapid.
  - UI: panou dedicat „Proces live” (Stop, Rollback, Edit Params, Resume).
  - Jurnal de schimbări parametri + scope de aplicare (doar viitor vs re-procesare bloc curent).

## Strategie – ordinea recomandată de implementare (roadmap)
1) Faza 1 – Fundament Stop/Resume (MVP) + gardieni ieftini
  - Atomic write `.part` → rename, `state.json`, inventar/curățare `.part` la start, determinare `next_chunk_index`.
  - Butoane GUI Stop/Resume + afișare stare minimă (chunk, ETA, param_set activ).
  - Gardieni ieftini: `ndt_exclude_recent_poses` (defazaj țintă), rafinare selectivă bugetată, telemetrie de bază + „traiectorie îmbogățită” CSV.
2) Faza 2 – Rollback + versionare parametri
  - `params_timeline.json`, `param_set_id` per‑chunk, GUI Edit Params + scope, ștergere sigură > X + re‑indexare.
3) Faza 3 – Snapshots RGD (resume rapid)
  - `rgd_snapshot_<i>.bin` opțional, încărcare la resume/rollback.
4) Faza 4 – Compatibilitate avansată + UX
  - Politici pentru parametri ce afectează segmentarea, indicator compatibilitate și asistent acțiuni în UI.
5) Extinderi opționale (după stabilizare):
  - Defazaj avansat la referință (praguri timp/distanță/rotație), automatizare CLI headless, analiză IMU ușoară pentru gating/adaptare.

 

## Contract și invariante
- Ordine temporală a datelor (IMU/LiDAR) și procesare strict în ordine (streaming linear pe chunk-uri).
- Chunk boundary = punct sigur (checkpoint); nu scriem rezultate parțiale în interiorul chunk-ului.
- Fiecare chunk este independent la nivel de persistență (fișiere + sidecar) și referințează param_set_id.
- Idempotent: reluarea nu trebuie să dubleze sau să altereze chunk-urile deja finalizate corect.

## Model de date pentru reluare și audit
- Per-chunk outputs (deja existente):
  - scan_lio_chunk_<i>.laz (și sidecar CSV opțional)
  - trajectory_lio_chunk_<i>.csv
- Metadata incrementală:
  - index_poses.json (listă sau per-chunk index_poses_chunk_<i>.json)
  - point_sizes_per_chunk.json (listă sau per-chunk)
- Session (poate fi reconstruit):
  - session.json, poses.reg, lio_initial_poses.reg
- Stare de proces (nou): `state.json`
  - version: string (compatibilitate)
  - session_id: uuid
  - next_chunk_index: int
  - last_global_pose_id / last_timestamp: număr
  - param_hash_active: sha256 al parametrilor activi
  - param_set_id_active: int
  - checkpoints: [ {chunk_index, timestamp, param_set_id, rgd_snapshot_path?} ]
  - created_at / updated_at
- Versiuni de parametri (nou): `params_timeline.json`
  - param_sets: { id -> {hash, values, author, note, created_at} }
  - changes: listă cronologică: {at_chunk_index, param_set_id, scope: "forward_only" | "reprocess_from_here"}
- Snapshots opționale (extins):
  - `rgd_snapshot_<i>.bin` la fiecare K chunk-uri (K configurabil) pentru resume rapid.

## Fluxuri de lucru

### 1) Stop sigur
- Trigger: utilizator sau `real_time_threshold_seconds`.
- Se finalizează chunk-ul curent (dacă a depășit stadiul „compute” și a ajuns în „write”, se finalizează scrierea) sau se abandonează chunk-ul curent (nimic nu este scris) în funcție de punctul în care ne aflăm.
- Se actualizează `state.json` (next_chunk_index, timestamps) și se închide.

### 2) Resume (crash-safe sau după Stop)
- La pornire: inventariază output_dir.
  - Elimină `.part` dacă există (fișiere temporare neterminate).
  - Determină `next_chunk_index` din fișierele complete existente (max(i) + 1).
  - Verifică `state.json` (dacă există) și congruența: `param_hash_active` ↔ parametrii încărcați, versiune software, calibrare.
- Dacă totul corespunde, reia de la `next_chunk_index`.
- Dacă nu corespunde (parametri/calibrare diferite), cere fie rollback, fie „start new session”.

### 3) Rollback
- Alegere din GUI/CLI: „revin la chunk X” (sau „șterge ultimele N chunk-uri”).
- Se șterg fișierele chunk-urilor > X și se ajustează incremental metadata (sau se regenerează din per-chunk metadata).
- `state.json.next_chunk_index = X + 1`; dacă există snapshot RGD pentru chunk X, îl încărcăm; altfel se reconstruiește harta activă din istoricul de traiectorie (cost CPU acceptabil ocazional).

### 4) Modificare parametri + continuare
- În GUI, editezi parametrii; sistemul calculează `param_hash_new` și persistă un `param_set_id_new` în `params_timeline.json`.
- Alegi scope-ul:
  - „doar înainte” (începând cu next_chunk_index, fără re-procesare retroactivă);
  - „re-procesează de la chunk X” (automat propus ca ultim checkpoint stabil sau chunk curent - 1).
- Se aplică Rollback dacă este cazul, apoi Resume cu `param_set_id_new`.
- Fiecare chunk scrie `param_set_id_used` în metadata pentru audit.

## Politici și restricții de compatibilitate parametri
- Parametri care afectează geometria chunk-urilor (ex.: `threshold_nr_poses`) după ce s-au scris chunk-uri:
  - Opțiunea 1 (MVP): interzis să-i schimbi fără a începe o sesiune nouă.
  - Opțiunea 2 (Extins): permiți schimbarea, dar impui rollback până la granița unde se poate resegmenta (de regulă de la zero sau de la un checkpoint dedicat).
- Parametri interni NDT/LO (epsilon, iterații, kernel etc.) sunt safe pentru schimbare cu rollback local.

## Persistență sigură și idempotentă
- Scriere atomică: la write, se folosește sufix `.part` (ex.: `scan_lio_chunk_42.laz.part`), apoi rename → fișier final.
- Idempotent: la start, fișiere `.part` se șterg; fișierele finale nu se rescriu.
- Session.json, poses.reg, lio_initial_poses.reg se regenerează din inventar la Stop/Finish (sau incremental, dar regenerabil oricând).

## Actualizări GUI (propuse)
- Panou „Proces live”:
  - Status: chunk curent, ETA, param_set_id activ, progres.
  - Butoane: Stop, Resume, Rollback (◄ 1, ◄ N, la index), Edit Params, Aplică (Forward / Reprocess from X).
  - Indicator compatibilitate (verde/galben/roșu) când schimbi parametri (dacă necesită rollback sau sesiune nouă).
- Panou „Parametri LO avansați”: vizualizare param_set-uri, istoric schimbări, duplicate/editare cu note.

## CLI (opțional)
- `--resume` (default dacă există output și state.json)
- `--rollback N` sau `--rollback-to X`
- `--apply-params FILE.toml --scope forward|reprocess-from X`
- `--no-snapshot-rgd` (dezactivează snapshots)

## Plan de implementare (faze + criterii de acceptare)

### Faza 1 (MVP – Stop/Resume/Write atomic)
- [ ] Writer atomic `.part` → rename pentru toate fișierele per-chunk.
- [ ] `state.json` minimal: next_chunk_index, param_hash_active, param_set_id_active, timestamps.
- [ ] Resume: inventar output_dir, curățare `.part`, determinare next_chunk_index, validare parametri.
- [ ] GUI: butoane Stop/Resume + afișare stare minimă (chunk, ETA, param_set activ).
- [ ] Regenerare `session.json`/`poses.reg`/`lio_initial_poses.reg` din inventar la Stop/Finish.
- [ ] Gardieni ieftini (runtime low-cost):
  - [ ] `ndt_exclude_recent_poses` (param în TOML/GUI) și integrare în ținta NDT.
  - [ ] Rafinare selectivă bugetată: +1 iterație și/sau `epsilon *= 0.5` când `confidence_score` < prag, în limita `max_refinement_time_ms_per_chunk`.
  - [ ] Telemetrie per-chunk: iters, stop_reason (converged|plateau|time_budget), scor NDT/inlier_ratio, `confidence_score` → CSV/JSON.
  - [ ] Traiectorie îmbogățită (CSV, ~10 Hz): match_score, inlier_ratio, iters, confidence_score.
- Acceptare:
  - [ ] Run → Stop → Resume produce rezultate identice cu rularea continuă; fără fișiere corupte.
  - [ ] În trafic urban, `ndt_exclude_recent_poses ∈ {1,2}` reduce potrivirea pe obiecte dinamice (observabil la scor/consistență) fără penalizare semnificativă de runtime.
  - [ ] Rafinarea selectivă respectă bugetul și îmbunătățește scorul pe chunk-urile sub prag (în medie) fără depășirea timpului maxim setat.
  - [ ] Fișierele de telemetrie și traiectorie îmbogățită sunt populate corect și rezistente la Resume.

### Faza 2 (Rollback + param_set versioning)
- [ ] `params_timeline.json` + param_set_id per-chunk.
- [ ] GUI: Edit Params + alegere scope (forward / reprocess from X).
- [ ] Rollback: ștergere sigură a chunk-urilor > X + re-sincronizare metadata.
- Acceptare: schimb parametri interni LO și re-procesez de la X; ieșirile sunt reproductibile și auditabile.

### Faza 3 (Checkpoints/snapshots RGD)
- [ ] Opțional, salvează `rgd_snapshot_<i>.bin` la fiecare K chunk-uri.
- [ ] Resume după rollback la X încarcă snapshot dacă disponibil, altfel reconstruiește.
- Acceptare: timp de reluare scade proporțional cu frecvența snapshot-urilor.

### Faza 4 (Compatibilitate avansată + UX)
- [ ] Politici pentru parametri care afectează segmentarea (ex.: interdicție sau rollback extins/sesiune nouă).
- [ ] UI: indicator compatibilitate și asistent de acțiune (sugerează ce rollback e necesar).
- Acceptare: schimbări incompatibile sunt semnalate clar și gestionate fără surprize.

## Testare (minimală)
- Unit: serializare/deserializare `state.json`, `params_timeline.json`, generare `param_hash` stabil.
- Integrare: 
  - Run → Stop → Resume (identic cu run continuu).
  - Run → crash la jumătatea unui write → Resume (fără corupție, cel mult se pierde chunk-ul curent).
  - Run → rollback X → reprocess from X cu parametri noi → rezultate persistă corect, audit OK.
- Performanță: overhead I/O pentru `.part` + rename neglijabil.

## Riscuri și mitigări
- Schimbare segmentare (chunking): limitați sau automatizați rollback până la o graniță sigură.
- Divergență RGD după param change: recomandare implicită de rollback cu 1–2 chunk-uri în plus.
- Output folder alterat manual: „Repair/Reindex” (regenerează session.json și metadata din fișiere per-chunk).

## Note de implementare
- Folosiți căi pe același volum pentru a garanta rename atomic pe Windows.
- Persistați `software_version`, `build_date`, `sensor_model` în `state.json` pentru trasabilitate.
- Includeți în `param_hash` doar câmpurile relevante LO/NDT (excludeți cele care nu afectează rezultat, ex.: UI-only).

---

Ultima actualizare: 2025-08-14
Autor: echipa HDMapping

## Anexe – scheme JSON minimale (pentru implementare rapidă)

Exemplu `state.json` (MVP):

```
{
  "version": "1.0",
  "session_id": "9b0c0c0e-7a2e-4f2d-9a4b-9d7d1c2e6a11",
  "software_version": "0.8.5",
  "build_date": "2025-08-12",
  "calibration_hash": "<sha256>",
  "param_hash_active": "<sha256>",
  "param_set_id_active": 1,
  "next_chunk_index": 17,
  "last_timestamp": 1.726.000.123.456,
  "updated_at": "2025-08-14T12:34:56Z"
}
```

Exemplu `params_timeline.json` (MVP):

```
{
  "param_sets": {
    "1": {
      "hash": "<sha256>",
      "values": { /* subset LO/NDT relevant */ },
      "author": "user",
      "note": "implicit",
      "created_at": "2025-08-14T10:00:00Z"
    },
    "2": {
      "hash": "<sha256>",
      "values": { /* modificări */ },
      "author": "user",
      "note": "tune pentru mișcări bruște",
      "created_at": "2025-08-14T11:00:00Z"
    }
  },
  "changes": [
    { "at_chunk_index": 17, "param_set_id": 2, "scope": "forward_only" }
  ]
}
```

Notă: `hash` se calculează pe o serializare stabilă (chei sortate) a subsetului de parametri care influențează rezultatul.

## Hook‑uri minime în cod (ghid implementare Faza 1)
- La start:
  - dacă există output_dir: curăță `.part`; detectează chunk-uri finale; stabilește `next_chunk_index`;
  - încarcă `state.json` (dacă există) și compară `param_hash_active`, `software_version`, `calibration_hash`.
- La începutul fiecărui chunk:
  - verifică dacă s-a cerut Stop (flag/GUI);
  - setează timer pentru `real_time_threshold_seconds` (doar pentru bucla LO principală).
- La scriere per-chunk:
  - scrie `.part` → flush → rename final; actualizează `point_sizes_per_chunk.json`/`index_poses.json`;
  - actualizează `state.json.next_chunk_index = i+1` și `updated_at`.
- La Stop/Finish:
  - regenerează `session.json`/`poses.reg`/`lio_initial_poses.reg` din inventar existent.

## Observații teoretice și poziționarea LO în pipeline-ul SLAM
- Despre abordare (LO din `lidar_odometry_step_1`): pragmatică și „directă”, dar cu elemente inteligente pentru robustitate/performanță:
  - Pipeline liniar pe chunk-uri, AHRS/IMU pentru inițializare, optimizare NDT/SF (și rigid) cu early‑stop și gardă de timp (`real_time_threshold_seconds` doar la bucla LO).
  - Fereastră rulantă pentru harta activă (RGD), filtrare intensitate, opțiune de consistență globală, scriere progresivă + resume.
  - Limite asumate: parametri constanți pe sesiune (în prezent), fără detecție automată de regim și fără închideri de buclă în Pasul 1.

- SLAM complet în pașii următori ai pipeline‑ului:
  - Pasul 2: `multi_view_tls_registration_step_2` execută SLAM intra‑sesiune cu loop closures și puncte GCP; aici se rezolvă corecțiile globale de deriva pe sesiune folosind factori suplimentari.
  - Pasul 3: `multi_session_registration_step_3` procesează rețeaua de sesiuni (rezultate din Pasul 2) cu un kernel robust, pentru aliniere multi‑sesiune și consistență globală la nivel de proiect.

- Implicații pentru prezentul plan (Stop/Rollback/Params/Resume):
  - Flexibilitatea de a opri/derula/edita parametri în Pasul 1 este utilă pentru a furniza intrări mai bune către Pasul 2 (unde se fac corecții globale prin loop closures/GCP).
  - Orice schimbare de parametri în Pasul 1 trebuie jurnalizată (param_set_id/hash) pentru trasabilitate când se analizează efectele în Pasul 2 și 3.

## Glosar termeni (referință rapidă)
- frame → în acest proiect corespunde conceptului de "pose" (poziție/atitudine la un timestamp). Cu Livox (scanare continuă) nu avem cadre discrete; lucrăm cu eșantionări temporale ale traiectoriei.
- pose → stare rigidă estimată la un timp (folosită ca unitate de lucru în LO); numărul de poze per chunk este controlat de `threshold_nr_poses`.
- keyframe (aprox.) → poză selectată/semnificativă folosită la actualizarea hărții active; în practică, pozele din fereastra rulantă ce alimentează RGD/NDT.
- chunk → grup de `threshold_nr_poses` poze consecutive + punctele aferente intervalului temporal; unitatea de procesare/salvare progresivă (`scan_lio_chunk_<i>.laz`, `trajectory_lio_chunk_<i>.csv`).
- index_pose → index [0..threshold-1] pentru poza asociată fiecărui punct din chunk (mapare punct → poză).
- scan (în output) → fișier LAZ per chunk; reprezintă agregatul de puncte din intervalul chunk-ului, nu un „frame” hardware discret.
- submap / hartă activă → structura RGD utilizată de NDT, menținută într-o fereastră rulantă controlată de `sliding_window_trajectory_length_threshold`.
- session → colecție de chunk-uri + metadata + traiectorii, finalizată în `session.json`, `poses.reg`, `lio_initial_poses.reg`.
- procesare progresivă / streaming → încărcare → optimizare → scriere imediată per chunk, opțional eliberare memorie; suportă Stop/Resume/Rollback.
- real_time_threshold_seconds → buget de timp doar pentru bucla LO principală; nu întrerupe buclele interne NDT (`optimize_sf2` / `optimize_rigid_sf`).

## Ce înseamnă „motion model” în codul nostru (LO Pasul 1)
Pe scurt: este un prior/regularizator 6DoF pe traiectorie, nu un model de oglindă al senzorului și nu un mecanism de discretizare a timestamp‑urilor.

- Unde este folosit
  - Optimizatoarele LO (NDT/SF/rigid_sf) primesc o traiectorie‑prior `intermediate_trajectory_motion_model` aliniată cu pozele intermediare și o folosesc ca termen de regularizare.
  - Corecția aplicată (dacă e cazul) este exportată pentru transparență în rezultate ca `motion_model_correction.{om,fi,ka}` (rotiri în grade: yaw=ka, pitch=fi, roll=om).

- Parametri și greutăți
  - 1‑sigma pozițional (metri): `lidar_odometry_motion_model_{x,y,z}_1_sigma_m`.
  - 1‑sigma unghiular (grade): `lidar_odometry_motion_model_{om,fi,ka}_1_sigma_deg`.
  - Greutăți interne: w = 1 / sigma²; pentru termeni unghiulari, sigmele în grade sunt convertite la radiani înainte de calcul.

- Cum tunezi (guideline pragmatic)
  - Sigme mai MARI ⇒ prior mai SLAB (mai multă libertate pentru date); sigme mai MICI ⇒ prior mai PUTERNIC (mai multă regularizare).
  - Punct de pornire rezonabil (de adaptat la scenă):
    - Translație (x,y,z): 0.5–1.0 m (mai mare în scene dinamice pentru a nu supra‑constrânge).
    - Roll/Pitch (om/fi): 2–5 deg (strânse dacă IMU este stabil; relaxează dacă zgomot mare/vibrații).
    - Yaw/Heading (ka): 5–10 deg (relaxează dacă observi drift sau probleme magnetice/observabilitate slabă în plan).
  - Practic OFF: setează 1‑sigma foarte mare (ex. 1e6) pe componentele pe care vrei să nu le regularizezi.

- Interacțiuni și note
  - Independent de defazajul țintei NDT (vezi secțiunea următoare) și de mecanica RGD; se combină aditiv în costul optimizării.
  - IMU/AHRS rămâne seed de orientare; priorul de mișcare doar stabilizează soluția între poziții consecutive.
  - Atenție la unități (m vs. deg) și la supra‑constrângeri în scene cu accelerații mari – pot „trage” soluția într‑o direcție greșită dacă sigmele sunt prea mici.

Tip: loghează/expune valorile efective ale greutăților (w) în telemetrie pentru debugging și tuning rapid.

## Mitigare obiecte dinamice: defazaj la referință (registrare cu lag)
Motivație: alinierea „poză curentă → poză anterioară imediat precedentă” poate supra‑potrivi structuri dinamice (mașini/persoane) deoarece deplasarea lor între cadre consecutive este mică. O referință cu defazaj (index/timp/distanță/rotație) crește paralaxa și robustețea.

Propuneri de politică (compatibile cu LO existent):
- MVP (simplu și robust): submap‑target fără ultimele M poze (excludere recenți).
  - Param: `ndt_exclude_recent_poses` (int, implicit 2) – când construiești/folosești RGD pentru matching, exclude contribuția ultimelor M poze față de poza curentă.
- Avansat (bazat pe praguri): alegi referința j < i pe baza schimbării minime:
  - `ref_min_pose_gap` (int, implicit 2–3)
  - `ref_min_time_delta_s` (implicit 0.2)
  - `ref_min_distance_m` (implicit 0.5)
  - `ref_min_rotation_deg` (implicit 5.0)
  - `ref_selection_mode`: `any_with_priority` (distance > rotation > time) sau `all` (toate pragurile);
  - `ref_candidate_window_back` (max câte poze în urmă căutăm, ex. 30), dar nu depășim fereastra RGD.

Integrare în compute (LO Pasul 2):
- Variante echivalente în cod:
  1) Când actualizezi/folosești RGD, nu include ultimele M poze (M = `ndt_exclude_recent_poses`). Ținta rămâne submap‑ul activ, dar fără contribuția celor mai recente poze.
  2) Dacă se implementează varianta cu praguri, determină j (referința) folosind traiectoria IMU/estimarea curentă; construiește/folosește submap ce acoperă până la j; dacă nu găsești j care respectă pragurile, aplică fallback: M dinamic crescut până când condiția este satisfăcută sau un M maxim.

Note practice:
- Menține compatibilitatea cu `sliding_window_trajectory_length_threshold`: excluderea ultimelor M poze se face înainte de construcția/consumul RGD pentru matchingul curent.
- Pentru evitarea regresei de acuratețe în scene lente, limitează M în [1..5] și activează numai în exterior/scene dinamice (un toggle în GUI).

Checklist implementare (incremental):
- [ ] Adaugă câmp `ndt_exclude_recent_poses` în `LidarOdometryParams` + TOML I/O + GUI (avansat).
- [ ] În Pasul 2, când agregi „harta activă” (RGD) sau când selectezi punctele țintă, exclude ultimele M poze față de poza curentă.
- [ ] Test: trafic urban – compară stabilitatea vs M={0,1,2,3}; verifică că runtime nu crește semnificativ.
- [ ] (Opțional) Adaugă parametrii `ref_min_*` și logica de selecție j pe baza IMU/traiectorie.
- [ ] Telemetrie: loghează j sau M ales per‑chunk (pentru analiză și tuning).

### Parametri noi (TOML/GUI) și exemplu
- `[lidar_odometry_control]`
  - `ndt_exclude_recent_poses` (int, implicit 0) — exclude ultimele M „poses” din ținta NDT pentru alinierea pozei curente (MVP).
  - `ref_min_pose_gap` (int, implicit 0) — diferență minimă de index între poză curentă și referință (experimental, avansat).
  - `ref_min_time_delta_s` (float, implicit 0.0) — defazaj minim în timp (s) (experimental).
  - `ref_min_distance_m` (float, implicit 0.0) — defazaj minim pe distanță (m) (experimental).
  - `ref_min_rotation_deg` (float, implicit 0.0) — defazaj minim pe rotație (deg) (experimental).
  - `ref_candidate_window_back` (int, implicit 0) — fereastră maximă de căutare în urmă (experimental).
  - `ref_selection_mode` (string: `any_with_priority` | `all`, implicit `any_with_priority`) — mod de selecție (experimental).

Exemplu TOML (activare MVP doar):

```toml
[lidar_odometry_control]
ndt_exclude_recent_poses = 2  # exclude ultimele 2 poze din submap-ul țintă
```

Exemplu TOML (avansat, experimental):

```toml
[lidar_odometry_control]
ndt_exclude_recent_poses = 1
ref_min_pose_gap = 3
ref_min_time_delta_s = 0.25
ref_min_distance_m = 0.7
ref_min_rotation_deg = 7.5
ref_candidate_window_back = 30
ref_selection_mode = "any_with_priority"
```

## Politică Step 1 (performanță): Odometrie‑only + „cheap guards”
Scop: viteză maximă și consum redus de resurse în Pasul 1. Orice ajustări globale se fac în Pașii 2–3.

Recomandări:
- Implicit, păstrează Pasul 1 ca odometrie locală (IMU + NDT/SF) fără operații costisitoare pe istoric.
- Activează doar gardieni ieftini (overhead ≈ zero):
  - Early‑stop + limite de iterații (LO/NDT) – deja implementat.
  - `real_time_threshold_seconds` – verificat între chunk‑uri (nu întrerupe sub‑buclile NDT).
  - Filtre ușoare la load (intensitate/radius) și decimare moderată pentru RGD.
  - Defazaj la referință (opțional): `ndt_exclude_recent_poses = 1..2` în scene dinamice; OFF implicit.
  - Telemetrie ușoară: log score NDT/inlier ratio/mărimea pasului – fără extra pase.

Evitați în Pasul 1 (mutat în 2–3):
- Consistency smoothing global care rescrie segmente mari.
- Loop closures, re‑optimizări globale sau analize de istoric în background.
- Reconstrucții extinse de submap în afara ferestrei rulante.

Sugestie UI (moduri):
- „Fast / Odom‑only” (default): gardieni ieftini ON, defazaj OFF.
- „Robust+” (opțional): permite `ndt_exclude_recent_poses` și alte praguri avansate; pașii 2–3 se ocupă de corecțiile globale (LC/GCP/multi‑sesiune).

## Analiză holistică IMU (ușoară, opțională)
Scop: extrage semnale ieftine din IMU înainte de procesare pentru a crește robustitatea fără cost mare de CPU.

Semnale extrase (O(N) pe lungimea IMU, memorie mică):
- Găuri în IMU (gaps) peste un prag (ms) și drift potențial.
- Staționări (ZUPT) – intervale în care accelerațiile/giroscopul indică mișcare ≈ zero.
- Traiectorie brută (coarsă) din IMU (AHRS + integ.) decimată, doar ca prior slab.
- Sanity calibrare: vector gravitație stabil, bias accel/gyro estimați, aliniere IMU ↔ senzor.
- Probleme de sincronizare timp (salturi/derapaj timestamp).
 - Nivel de dinamică și jerk (|a−g|, |ω| și derivate) – clasificare low/med/high.
 - Detecție de viraj (turns) – prag pe norm(ω) sau pe componente.
 - Vibrații/șocuri (RMS de bandă înaltă pe accelerații).
 - Saturații/clipping (depășire praguri hardware accel/gyro).
 - Drift de heading (yaw) estimat în timp.
 - Panta/înclinarea terenului (slope/tilt) din direcția gravitației.

Output (fișier auxiliar): `imu_insights.json`
- `gaps`: [{start_ts, end_ts, dt_ms}]
- `stationarity`: [{start_ts, end_ts}]
- `imu_bias`: {accel: [bx,by,bz], gyro: [bx,by,bz]}
- `gravity`: {norm: g, direction: [gx,gy,gz]}
- `coarse_trajectory_decimated`: [{ts, T_world_imu}] (opțional)
- `time_sync_issues`: [{ts, type, detail}]
 - `dynamics_level`: [{ts, level: "low|med|high"}]
 - `turns`: [{start_ts, end_ts, omega_peak}]
 - `vibration_rms`: [{start_ts, end_ts, rms}]
 - `clipping_events`: [{ts, sensor: "accel|gyro", axis, value}]
 - `heading_drift_rate_deg_per_s`: float (estimare globală)
 - `slope_tilt_deg`: {pitch: deg, roll: deg}
 - `suggested_overrides` (opțional): { ndt_exclude_recent_poses, decimation, epsilon_scale, rigid_sf_toggle }

Integrare în Pasul 1 (fără overhead semnificativ):
- ZUPT gating: în intervalele de staționare, micșorează iterațiile/epsilon sau sare actualizări (hold‑pose), reducând zgomotul.
- Defazaj auto: dacă mișcare mică (din IMU), crește `ndt_exclude_recent_poses` la 2–3; altfel revine la 0–1.
- Fereastră rulantă adaptivă: crește ușor `sliding_window_trajectory_length_threshold` când dinamica crește.
- Prior slab: folosește traiectoria coarsă ca inițializare, cu greutate mică (nu forțează soluția).
- Alerte: dacă apar gaps mari, log + posibil avertisment în GUI.

Indicator → acțiune (gating/adaptare low‑cost):
- Stationarity (ZUPT) → hold‑pose sau reduce iterații/epsilon pentru update‑uri mici.
- Dynamics/Jerk high → crește `ndt_exclude_recent_poses`, mărește decimarea/voxel, întărește kernel robust.
- Turn detection → comută tactici pentru rotații mari (ex. favorizează rigid_sf); micșorează pașii.
- Vibration RMS high → crește decimarea și voxel, down‑weight reziduuri instabile.
- Clipping events → sare update sau down‑weight în ferestrele afectate; log pentru QA.
- Time sync issues → avertisment; relaxare temporară a încrederii în priors.
- Heading drift ↑ → relaxează priorul pe yaw; marchează chunk „low confidence”.
- Slope/Tilt stabil → constrângeri pe verticală/elevație (căutare limitată), inițializare mai bună.

Parametri (TOML) propuși:
- `[imu_analysis]`
  - `enable` (bool, implicit true)
  - `gap_threshold_ms` (int, implicit 50)
  - `stationarity_min_duration_s` (float, implicit 0.5)
  - `zupt_enable` (bool, implicit true)
  - `zupt_accel_sigma` (float, implicit 0.05)
  - `zupt_gyro_sigma` (float, implicit 0.01)
- `[lidar_odometry_control]`
  - `adapt_from_imu` (bool, implicit false) – permite ajustarea pragurilor LO/NDT din semnalele IMU.
  - `defazaj_auto_from_imu` (bool, implicit false) – permite setarea dinamică a `ndt_exclude_recent_poses`.
- `[performance]`
  - `imu_prepass_time_budget_ms` (int, implicit 100) – buget soft; dacă e depășit, se reduce granularitatea decimării/colectării.

Exemplu TOML:

```toml
[imu_analysis]
enable = true
gap_threshold_ms = 50
stationarity_min_duration_s = 0.5
zupt_enable = true

[lidar_odometry_control]
adapt_from_imu = true
defazaj_auto_from_imu = true

[performance]
imu_prepass_time_budget_ms = 100
```

Checklist implementare (rapid):
- [ ] Pre‑pass pe IMU (O(N)): detectează gaps/staționări, estimează bias/gravitație; scrie `imu_insights.json` (opțional).
- [ ] Dacă `adapt_from_imu`: aplică ZUPT gating și setări ușoare (epsilon/iterații) pe chunk‑urile afectate.
- [ ] Dacă `defazaj_auto_from_imu`: setează `ndt_exclude_recent_poses` în funcție de dinamica locală.
- [ ] Afișează sumar în GUI (toggle) fără a ține thread‑ul de compute ocupat; calculul e rapid.
 - [ ] Telemetrie per‑chunk: dynamics_level, turns, vibration_rms, clipping_count, time_gaps, heading_drift.

## Viteză vs. Calitate: profiluri și rafinare selectivă (opțional)
Obiectiv: păstrăm Pasul 1 rapid, dar îmbunătățim calitatea acolo unde semnalele indică risc, sub un buget strict.

Profiluri (TOML `[performance]`):
- `profile = "fast" | "balanced" | "quality"` (implicit: `fast`)
  - fast: odometrie‑only + guards ieftini (vezi secțiunea dedicată); fără rafinare.
  - balanced: rafinare selectivă doar pentru chunk‑uri “low confidence”, sub buget strict.
  - quality: rafinare pentru mai multe chunk‑uri, praguri mai stricte (cost mai mare).

Scor de încredere per‑chunk (calcul ieftin, log în telemetrie):
- Semnale: NDT score/inlier ratio, RMS reziduuri, mărime pas (delta), dinamica IMU (jerk/turns), gaps/sync issues.
- `confidence_score ∈ [0,1]` (ex.: 1 = foarte bun); `confidence_low_threshold` (implicit 0.4).

Rafinare selectivă (dacă `profile` ≠ fast și `confidence_score < threshold`):
- Ajustări locale pentru chunk curent:
  - crește `nr_iter` LO (limită) și/sau scade `lo_convergence_epsilon` cu un factor (ex. ×0.5);
  - reduce decimarea/voxel pentru RGD cu un pas (mai dens, dar limitat);
  - activează `rigid_sf` dacă era off; `ndt_exclude_recent_poses = max(M,1)` în scene dinamice;
  - păstrează fereastra rulantă neschimbată (nu mări memoria).
- Bugete:
  - `max_refinement_time_ms_per_chunk` (implicit 0 – dezactivat)
  - `global_refinement_time_budget_s` (implicit 0 – dezactivat)
  - oprire imediată dacă se depășește bugetul.

Parametri TOML propuși:
```toml
[performance]
profile = "balanced"                 # fast|balanced|quality
max_refinement_time_ms_per_chunk = 50 # buget local pe chunk
global_refinement_time_budget_s = 10  # buget total

[lidar_odometry_control]
confidence_low_threshold = 0.4        # declanșează rafinarea selectivă
refine_iters_boost = 1                # +1 iterație LO (plafon)
refine_epsilon_scale = 0.5            # epsilon *= 0.5 (mai strict)
refine_enable_rigid_sf = true         # pornește rigid_sf în rafinare
refine_decimation_step = 1            # micșorează decimarea cu un pas
```

GUI (sugestie):
- Afișaj status per‑chunk: verde (ok), galben (rafinat), roșu (low confidence nerefinat / buget epuizat).
- Toggle profile + bugete; jurnal cu ce ajustări s‑au aplicat.

Note:
- Rafinarea e locală și scurtă; nu re‑optimizăm retroactiv. Corecțiile globale rămân pentru Pasul 2/3.
- Dacă bugetele sunt 0, comportamentul rămâne identic cu “fast”.

### Early‑stop pe palier (iterații)
Scop: oprim iterațiile când progresul devine neglijabil sau stagnat, pentru a economisi timp fără a sacrifica calitatea.

Parametri existenți (LO și sub‑pași):
- LO: `lo_convergence_epsilon`, `lo_min_relative_improvement`, `lo_max_no_improve_iters` (în `LidarOdometryParams`).
- Sub‑pași: `sf2_convergence_epsilon`, `rigid_sf_convergence_epsilon` (pentru optimizările secundare).

Regulă (per iterație):
- Calculează `delta_norm` și/sau `relative_improvement` al costului.
- Convergență dacă `delta_norm < lo_convergence_epsilon` sau `relative_improvement < lo_min_relative_improvement` pentru `lo_max_no_improve_iters` la rând.
- În profil `fast`: set praguri mai agresive (K mai mic). În `quality`: K mai mare și epsilon mai strict.

Telemetrie și jurnal:
- Per‑chunk: `iters_done`, `no_improve_streak_max`, `final_stop_reason = converged|plateau|time_budget`.
- Log pentru sub‑pași: iterații și motiv oprire (sf2/rigid_sf).

TOML (exemplu):
```toml
[lidar_odometry_control]
lo_convergence_epsilon = 1e-12
lo_min_relative_improvement = 0.004
lo_max_no_improve_iters = 3

[ndt_substeps]
sf2_convergence_epsilon = 1e-6
rigid_sf_convergence_epsilon = 1e-6
```

## Automatizare Pasul 1 (headless/batch)
Scop: rulare neasistată, reluabilă, cu bugete de timp și control calitate.

Contract minimal:
- Intrări: set senzori (Lidar + opțional IMU), fișier config TOML.
- Ieșiri: poses.csv/trajectory.xyz (progresiv), `state.json`, `imu_insights.json` (dacă activ), log execuție.
- Erori: cod ieșire ≠ 0, raport sumar JSON cu cauză/locație.

Pași de execuție (headless):
1) Validare rapidă intrări: existență fișiere, calibrare, permisiuni scriere.
2) Inițializare sesiune: `run_id`, directoare output, scriitor atomic.
3) Resume: dacă există `state.json` valid și output parțial, reconciliază și stabilește `next_chunk_idx`.
4) Bugete: aplică `real_time_threshold_seconds`, `max_refinement_time_ms_per_chunk`, `global_refinement_time_budget_s` (opțional) din config.
5) Loop principal per‑chunk:
  - Citește/formează chunk; calculează scor încredere ieftin.
  - Rulează odometrie; dacă sub prag și profil ≠ fast, aplică rafinare selectivă în limita bugetului.
  - Mitigare dinamice: `ndt_exclude_recent_poses` conform policy.
  - Salvare progresivă + `state.json` actualizat (atomic); telemetrie per‑chunk.
  - Early‑stop dacă depășește buget/limită timp.
6) Finalizare: închide atomic outputs, scrie sumar execuție (durată, chunk‑uri, rafinări, avertismente).
7) Returnează cod ieșire și calea către raportul sumar.

Batch (mai multe seturi):
- Coada de job‑uri cu limite de concurență (CPU/IO), continue‑on‑error, raport agregat.
- Per‑job: izolare `run_id`/foldere, aceleași pași ca mai sus.

Config cheie (TOML):
- `[performance]`: `profile`, `max_refinement_time_ms_per_chunk`, `global_refinement_time_budget_s`.
- `[lidar_odometry_control]`: `confidence_low_threshold`, `refine_*`, `ndt_exclude_recent_poses`.
- `[resume]`: activare, politică rollback minim, sane‑checks outputs.
- `[io]`: căi intrare/ieșire, politică de rotație log, compresie (opțional).

Observații:
- Automatizarea nu schimbă logica de bază; doar o orchestrează sigur, cu reziliență și raportare.
- Pasul 1 rămâne odometrie‑centric; corecții globale în Pasul 2/3.

## Detectare și clasificare obiecte în mișcare (opțional, bugetată)
Scop: identificăm puncte/clus­tere dinamice în timpul registrării cadrelor, pentru a le filtra înainte de salvare sau a le eticheta pentru pași ulteriori. Respectăm profilul de performanță și bugetele de timp.

MVP (ieftin, pe chunk):
- După estimarea poziției chunk‑ului curent, transformă punctele în același reper ca referința (hartă/fereastră rulantă).
- Voxelizare rapidă (voxel_size_ms) a țintei statice (fereastra rulantă fără ultimele M poziții) pentru coerență temporală.
- Pentru fiecare punct al chunk‑ului: calculează un scor de mișcare ieftin:
  - reziduu distanță la suprafața/centroidul voxelului vecin (sau la NDT cell) > prag;
  - opțional: inversia semnului pe vectorul razei (range‑rate aproximat) între T și T‑1 pe același azimut/beam;
  - penalizează punctele în zone cu variație mare între ultimele K cadre (instabilitate temporală).
- Marchează punctele cu scor peste prag drept „dynamic_candidates”.
- Clusterează (DBSCAN ieftin pe subset) doar dacă `label_output = "clusters"` și bugetul permite.

Integrare pipeline (condiționată de profil/buget):
- fast: dezactivat.
- balanced: rulează doar când `confidence_score < threshold` sau `dynamics_level` IMU este ridicat; limitează la `max_motion_segmentation_ms_per_chunk`.
- quality: rulează constant în limitele bugetului; permite clustering.
- Filtrare: dacă `filter_before_save = true`, exclude `dynamic_candidates` din ieșirile punctuale (doar pentru acest chunk) înainte de scrierea progresivă.
- Etichetare: dacă `label_output != "none"`, scrie mască per‑punct sau liste de clustere pentru consum ulterior (Pasul 2/3 sau post‑procesare).

Parametri TOML propuși:
```toml
[motion_segmentation]
enabled = false
mode = "balanced"                   # fast|balanced|quality (suprascrie local profilul de performanță)
max_motion_segmentation_ms_per_chunk = 8
temporal_window = 3                  # K cadre pentru coerență temporală
exclude_recent_poses_M = 2           # re‑folosește politica de defazaj (evită ținta imediată)
voxel_size_ms = 0.20                 # m (downsample la detecție)
residual_threshold_m = 0.25          # prag pentru scor de mișcare
min_cluster_points = 30              # pentru clustering opțional
label_output = "none"               # none|per_point_mask|clusters
filter_before_save = false           # exclude din ieșirile punctuale ale chunk‑ului
```

Ieșiri (opțional):
- `chunk_[i]_dynamic_mask.bin` (bitmask per‑punct) sau `chunk_[i]_clusters.json` (bbox, centroid, număr puncte, scor mediu).
- Telemetrie: `dynamic_ratio`, `num_clusters`, `avg_cluster_size`, timp procesare ms, motive declanșare (low_confidence/dynamics_level/budget_free).

Calitate vs. timp (guideline):
- O(N) pe puncte cu voxelizare și eșantionare; menține subsampling pentru balanced.
- Nu afectăm optimizarea de odometrie a chunk‑ului curent; doar filtrăm/etiche­tăm puncte la ieșire și furnizăm semnale pentru pașii următori.

Note:
- Clasificarea în „dinamic/static/nesigur” este suficientă în Pasul 1; identificarea semantică se lasă pentru pașii ulteriori.
- Dacă `enabled=true` dar bugetul este epuizat, se sare peste detecție (nu blocăm bugetul global de rafinare/LO).

## Exporturi: traiectorie, calitate, nor „asamblat” (opțional, bugetat)
Scop: pe lângă output‑urile implicite, permitem exporturi suplimentare controlate din TOML, cu bugete ca să nu încetinim semnificativ Pasul 1.

Tipuri de export:
- Traiectorie (standalone): CSV, TUM, JSON (quat + t, timestamps).
- Calitate (input + procesare + rezultat): sumar JSON și CSV per‑chunk.
- Nor „asamblat” (agregat): pe segmente (frames/secunde) sau întreg, cu voxelizare/decimare și format LAZ/PLY.

Parametri TOML propuși:
```toml
[exports]
enable = true
time_budget_s = 15                  # buget total pentru exporturi (în afara bugetelor LO)

[exports.trajectory]
enable = true
formats = ["csv", "tum", "json"]  # subset: csv|tum|json

[exports.quality]
enable = true
include_input_stats = true          # Lidar/IMU: gaps, rate, histograme
include_odometry_stats = true       # iterații, epsilon atins, scoruri, timpi
include_motion_stats = true         # dynamic_ratio, clustere (dacă activ)
per_chunk_csv = true

[exports.pointcloud]
assembled_enable = false
assembled_format = "laz"            # laz|ply
assembled_segment_mode = "frames"   # none|frames|seconds|full
assembled_frames_per_segment = 500
assembled_seconds_per_segment = 15.0
assembled_voxel_m = 0.10            # downsample pentru scriere
assembled_max_points_million = 200  # limită siguranță per segment
assembled_colorize = "intensity"    # none|intensity (dacă disponibil)

chunks_enable = false               # salvare per‑chunk
chunks_format = "laz"               # laz|ply
chunks_decimation = 2               # 1 = nu decima
```

Structură directoare (exemplu):
```
outputs/<run_id>/
  trajectory/
    trajectory.csv
    trajectory.tum
    trajectory.json
  quality/
    summary.json
    per_chunk_metrics.csv
    imu_insights.json            # dacă activ
    motion_stats.csv             # dacă activ
  pointcloud/
    assembled/
      segment_000.laz
      segment_001.laz
      ... sau full.laz           # dacă mode=full
    chunks/                      # dacă chunks_enable=true
      chunk_000.laz
      chunk_001.laz
```

Calitate: ce măsurăm și raportăm
- Input Lidar: număr cadre, puncte/ cadru (min/med/max), histograme distanță/intensitate, dropouts estimat.
- Input IMU: rata eșantionare, gaps > X ms, varianța accel/gyro, drift heading preliminar.
- Odometrie: iterații medii, timp per‑chunk, scoruri NDT/inliers, epsilon atins, `confidence_score` mediu, early‑stops.
- Dinamică: `dynamic_ratio`, `num_clusters`, timp detecție (dacă activ).
- Ieșire: lungime traiectorie, smoothness (jerk angular/linear), bounding box, densitate nor „asamblat”.

Integrare și performanță
- Exporturile rulează incremental (după fiecare chunk sau la prag de segment) și folosesc scriere atomică.
- Respectă `exports.time_budget_s`; dacă e epuizat, exporturile non‑critice sunt sărite (nu afectează LO).
- „Asamblarea” nori­lor folosește pozițiile estimate; nu modifică estimarea (doar agregă).
- Pentru datasets mari, preferă segmente și `assembled_voxel_m` > 0.10 pentru a limita IO/timp.

Reluare (resume)
- Traiectorie: idempotentă; rescriere completă sau append sigur cu verificare `last_chunk_idx`.
- Calitate: CSV per‑chunk ușor de continuat; `summary.json` recalculat la final.
- Nor „asamblat”: segmente indexate (000,001,…) cu metadate (range chunk_idx); se re‑scrie doar segmentul curent.

### Traiectorie îmbogățită: scalari de încredere și dinamică (recomandat)
Obiectiv: includem în traiectorie indicatori utili pentru pașii următori (control calitate, gating, prioritizare loop‑closures), calculați ieftin și, când e posibil, alimentați de IMU.

Surse și principii
- Viteze/accelerații:
  - `velocity_source = imu | pose_diff` (implicit: `imu` dacă disponibil; altfel derivat din poziții succesive + filtru).
  - Cadrul: `frame = world | body` (implicit: `world`).
  - Netedizare: filtru `ewma` sau medie mobilă; evităm zgomotul la 100 Hz.
- Oscilații locale pe ferestre multiple (ex.: 0.5s, 1s, 3s):
  - măsuri ieftine: RMS, deviație standard, rata de treceri prin zero (ZCR) pentru yaw‑rate/gyro.
  - opțional estimări frecvență dominantă (peak simplu) dacă bugetul permite.
- Încredere registrare locală: `confidence_score` (vezi secțiunea profiluri), `match_score/ndt_score`, `inlier_ratio`, `residual_rms`, `iters`, `epsilon_reached`.
- Semnale IMU (dacă pre‑procesarea este activă): `zupt_flag`, `dynamics_level`, `vibration_rms`, `clipping_count`.

Parametri TOML propuși
```toml
[exports.trajectory.enriched]
enable = true
velocity_source = "imu"            # imu|pose_diff
frame = "world"                    # world|body
sample_rate_hz = 10                 # resampling pentru scalari (evită fișiere uriașe)
smooth = { type = "ewma", tau_s = 0.5 }
windows_s = [0.5, 1.0, 3.0]
include_frequency_estimates = false # calculează peak aproximativ (cost suplimentar)
fields = [
  "confidence_score", "match_score", "inlier_ratio", "residual_rms",
  "iters", "epsilon_reached", "dynamics_level", "zupt_flag",
  "v_lin", "a_lin", "omega",
  "osc_rms", "osc_std", "osc_zcr",
  "dynamic_ratio", "num_clusters",
  "imu_gap_ms", "lidar_gap_ms", "imu_clip_cnt", "lidar_drop_ratio"
]
```

Câmpuri propuse (per rând în traiectorie)
- Timp și poză: `t, tx, ty, tz, qx, qy, qz, qw`.
- Încredere LO: `confidence_score, match_score, inlier_ratio, residual_rms, iters, epsilon_reached`.
- Dinamică (în `frame` ales):
  - viteză liniară `v_lin_x,y,z` [m/s]
  - accelerație liniară `a_lin_x,y,z` [m/s^2]
  - viteză unghiulară `omega_x,y,z` [rad/s]
- Oscilații (pentru fiecare fereastră w ∈ `windows_s`):
  - `osc_rms_gyro_w`, `osc_rms_acc_w`
  - `osc_std_yaw_rate_w`
  - `osc_zcr_yaw_rate_w` (treceri prin zero/s)
- IMU/Calitate: `zupt_flag, dynamics_level, vibration_rms, clipping_count, imu_gap_ms`.
- Lidar/Calitate: `lidar_gap_ms, lidar_drop_ratio`.
- Dinamică scenă: `dynamic_ratio, num_clusters` (dacă motion_segmentation activă).

Exemplu CSV (header)
```
t,tx,ty,tz,qx,qy,qz,qw,confidence_score,match_score,inlier_ratio,residual_rms,iters,epsilon_reached,
v_lin_x,v_lin_y,v_lin_z,a_lin_x,a_lin_y,a_lin_z,omega_x,omega_y,omega_z,
osc_rms_gyro_0p5,osc_rms_gyro_1p0,osc_rms_gyro_3p0,osc_zcr_yaw_rate_0p5,osc_zcr_yaw_rate_1p0,osc_zcr_yaw_rate_3p0,
zupt_flag,dynamics_level,vibration_rms,clipping_count,imu_gap_ms,lidar_gap_ms,lidar_drop_ratio,
dynamic_ratio,num_clusters
```

Exemplu JSON (schemă per înregistrare)
```json
{
  "t": 123.456,
  "pose": {"t": [tx,ty,tz], "q": [qx,qy,qz,qw]},
  "lo": {"confidence": 0.82, "match_score": 0.65, "inlier_ratio": 0.83, "residual_rms": 0.07, "iters": 7, "epsilon_reached": true},
  "motion": {
    "frame": "world",
    "v_lin": [0.12, -0.03, 0.00],
    "a_lin": [0.4, 0.2, -0.1],
    "omega": [0.00, 0.00, 0.03]
  },
  "osc": {
    "0.5": {"gyro_rms": 0.02, "yaw_zcr": 0.8},
    "1.0": {"gyro_rms": 0.015, "yaw_zcr": 0.6},
    "3.0": {"gyro_rms": 0.01, "yaw_zcr": 0.2}
  },
  "imu": {"zupt": false, "dynamics_level": 1, "vibration_rms": 0.02, "clipping_count": 0, "gaps_ms": 0},
  "lidar": {"gaps_ms": 0, "drop_ratio": 0.01},
  "scene": {"dynamic_ratio": 0.05, "num_clusters": 2}
}
```

Implementare (ieftină, incrementală)
- Dacă `velocity_source=pose_diff`: derivăm viteză/acc din delta‑pose finite, apoi netezim (`ewma`).
- Dacă `velocity_source=imu`: folosim IMU sincronizat; `a_lin` corectat pentru gravitație dacă există aliniere; `omega` direct din giroscop.
- Oscilații: calcul incremental pe ferestre glisante (ring‑buffer) cu RMS/ZCR; evităm FFT implicit.
- Încredere LO: citim scoruri/iterații din odometrie; normalizăm în `confidence_score`.
- Scriere: resampling la `sample_rate_hz`, atomic append; compatibil cu resume.

Note
- Unități: m/s, m/s^2, rad/s; timpi în secunde; ferestre definite în secunde reale (nu în cadre).
- Dacă nu există IMU, câmpurile aferente lipsesc sau sunt `null`; se folosește `pose_diff` pentru viteză/acc.
- TUM clasic rămâne disponibil (fără câmpuri extra); CSV/JSON conțin câmpurile îmbogățite.

## Auto‑parametrizare și Automatizare bazată pe metrici (opțional, sigur)
Scop: folosim metricile colectate (încredere, reziduuri, dinamica IMU, oscilații, gaps, dinamica scenei) ca să alegem parametri adecvați sau să ajustăm fin, cu limite stricte pentru a păstra viteza și reproductibilitatea.

Moduri:
- `assist`: doar recomandă parametri (nu schimbă în runtime); scrie `params_suggestions.json`.
- `offline`: rulează un pilot (ex. 5–10% din date) → calculează recomandări → reluare completă cu parametrii recomandați.
- `online`: ajustări mici, locale, în runtime, cu cooldown și jurnal `params_timeline.json`.

Guardrails (obligatorii):
- Liste albe de parametri ajustabili; intervale min/max; pas de schimbare; cooldown (ex. 10 chunk‑uri între modificări pe același parametru).
- Bugete de timp specifice auto‑tune (nu pot depăși bugetele LO/refinement).
- Reversie: dacă 3–5 chunk‑uri consecutive sunt „bune”, revine gradual la valorile de bază.

Parametri TOML propuși:
```toml
[auto_tune]
enabled = false
mode = "assist"                 # assist|offline|online
pilot_fraction = 0.08            # pentru offline
cooldown_chunks = 10
max_changes_per_minute = 4
allowed_params = [
  "lo_convergence_epsilon", "lo_max_iters",
  "rgd_decimation", "ndt_exclude_recent_poses",
  "refine_enable_rigid_sf", "refine_decimation_step"
]

[auto_tune.clamps]
lo_convergence_epsilon = { min = 1e-5, max = 5e-3, step = 0.5 }
lo_max_iters = { min = 4, max = 20, step = 1 }
rgd_decimation = { min = 1, max = 4, step = -1 }      # -1 înseamnă face mai dens
ndt_exclude_recent_poses = { min = 0, max = 5, step = 1 }
```

Reguli de decizie (exemple, ieftine):
- Încredere scăzută (`confidence_score < 0.4`) sau `residual_rms` mare:
  - scade `lo_convergence_epsilon` (×0.5 în limite) sau crește `lo_max_iters` (+1), după cooldown.
- `dynamic_ratio` mare sau `dynamics_level` IMU ridicat:
  - crește `ndt_exclude_recent_poses` (max 5); activează `refine_enable_rigid_sf` pentru chunk‑ul curent (dacă buget).
- Oscilații ridicate (RMS/ZCR pe yaw‑rate) și drift local:
  - micșorează ușor decimarea RGD (`rgd_decimation -= 1` în limite) pentru mai multă structură; plafonează mărimea pasului (delta pose clamp, dacă există hook).
- ZUPT/stationary:
  - reduce temporar `lo_max_iters` (nu irosi buget) și crește pragurile pentru acceptarea convergenței.
- Gaps mari IMU/Lidar:
  - preferă `velocity_source=pose_diff`; dezactivează temporar rafinări costisitoare.

Pipeline:
- assist: după pilot scrie `params_suggestions.json` cu valorile recomandate și justificare (metrică → schimbare).
- offline: reconfigurează și re‑rulează complet; marchează `config_origin = "auto_offline"` în sumar.
- online: aplică numai schimbări locale cu jurnal în `params_timeline.json` (timestamp, chunk_idx, param, vechi→nou, motiv, metrici relevante).

Recomandări inițiale (alegere profil):
- Profil `fast` dacă: scene statice, `confidence_score` > 0.7 în pilot, dinamică redusă, gaps mici.
- Profil `balanced` dacă: `dynamic_ratio` moderat sau `confidence_score` variabil; activează rafinare selectivă.
- Profil `quality` dacă: cerințe de acuratețe crescute sau pilotul indică reziduuri mari constant (atenție la timp).

Reproductibilitate:
- Toate deciziile auto‑tune sunt deterministe pe baza metricilor logate; jurnal complet pentru refacere.
- Exportă `auto_decisions_summary.json` cu statistici (număr schimbări/parametru, impact estimat, timp consumat).

## Reprocesare segment (manuală sau bazată pe metrici)
Scop: re‑executăm Pasul 1 pe un interval precis (chunk‑uri sau timp) după o analiză directă ori semnale din metrici, apoi „îmbinăm” rezultatele în ieșirile existente fără a compromite siguranța și trasabilitatea.

Modalități de selectare a segmentului
- `chunk_range`: [start_idx, end_idx] incluziv.
- `time_range_s`: [t0, t1] secunde (convertește la chunk‑uri).
- `distance_range_m`: [d0, d1] (opțional, dacă există acumulare distanță).
- Context: `warmup_chunks` înainte de start (pentru continuitate) și `cooldown_chunks` după end (smoothing/stitch).

Strategii de îmbinare (splice)
- `local_splice_no_cascade` (rapid): reprocesează doar segmentul (+warmup/cooldown scurte). Posibilă mică discontinuitate la capete; marchează în raport.
- `splice_with_cascade` (sigur): reprocesează segmentul și recalculează toate chunk‑urile ulterioare până la final (cost mai mare, consistență maximă).

Override parametri (opțional)
- Se pot aplica suprascrieri locale de parametri doar pentru intervalul vizat (ex.: `ndt_exclude_recent_poses`, `lo_max_iters`, `refine_*`, `motion_segmentation.*`).
- Toate schimbările sunt jurnalizate în `params_timeline.json` cu motiv și metrici asociate.

Parametri TOML propuși
```toml
[reprocess]
enabled = false
mode = "local_splice_no_cascade"   # local_splice_no_cascade|splice_with_cascade
select = { type = "chunk_range", start = 120, end = 260 }
warmup_chunks = 3
cooldown_chunks = 3
override = { lo_max_iters = 12, ndt_exclude_recent_poses = 2 }
time_budget_s = 120                 # limitează durata re‑procesării
dry_run = true                      # doar estimează costul și impactul
```

Flux operațional
1) Dry‑run: validează intervalul, estimează timp/cost IO, detectează dependențe (ex.: exporturi afectate).
2) Checkpoint: snapshot metadate și indexări segment pentru rollback rapid.
3) Rollback parțial: taie ieșirile la începutul segmentului (traj., quality per‑chunk, segmente pointcloud) folosind scriere atomică.
4) Reprocesare: rulează Pasul 1 cu `override` în fereastra selectată; aplică `warmup/cooldown` pentru continuitate.
5) Splice: scrie ieșirile noi și reface indexările/segmentele afectate; dacă `splice_with_cascade`, continuă până la final.
6) Finalizare: scrie raport `reprocess_summary.json` (interval, motiv, param overrides, timp, impact), plus actualizări în `state.json`.

Ieșiri și trasabilitate
- `reprocess_manifest.json`: definiția segmentului, strategia, overrides, profil vechi/nou.
- `reprocess_summary.json`: timp total, număr chunk‑uri, diferențe statistice (reziduuri, confidence, drift), segmente rescrise.
- Marcaje în traiectorie/quality: tag „reprocessed=true” pe rândurile din interval.
- Opțional: păstrează un backup incremental pentru undo rapid (stocare rotativă, limită GB).

Considerații de performanță
- Preferă `local_splice_no_cascade` când salturile la capete sunt mici (verificare automată pe delta pose și reziduuri la granițe).
- Refolosește cache‑uri per‑chunk dacă există (downsample, nor intermediar) pentru a reduce IO.
- Respectă `time_budget_s`; dacă este depășit, se face „commit” până la punctul atins și se scrie un avertisment.

## Index de implementare (cod + secțiuni) – lidar_odometry_step_1_enhanced00
Scop: referințe precise din cod pentru fiecare punct al strategiei (unde este deja implementat, unde adăugăm hooks, ce rămâne TODO).

Context proiect
- Enhanced app: `apps/lidar_odometry_step_1_enhanced00/`
  - `lidar_odometry.cpp` – implementare extinsă (streaming, load, compute, save, exports JSON/TOML)
  - restul fișierelor forward la original (GUI, utils, optimizatori, TOML IO)
- Original app: `apps/lidar_odometry_step_1/` – majoritatea utilitarelor/optimizatorilor

Delta principal (enhanced00 vs original)
- Streaming per‑set: `run_lidar_odometry_streaming(...)` și `save_streaming_chunk(...)` [enhanced]
- Exporturi suplimentare la final: `save_parameters_toml(...)`, `save_processing_results_json(...)` [enhanced]
- Filtre intensitate la load/compute: `intensity_ignore_ranges_*` + `set_intensity_load_ranges(...)` [enhanced]
- Progressive save integ.: `save_result(...)` copiază LAZ‑uri progresive și scrie traiectorii chunk [enhanced]
- Indexări/metadata: `index_poses.json`, `point_sizes_per_chunk.json`, `session.json` [enhanced]

1) Procesare flexibilă, resume, atomic writes
- Plan: secțiunile „Automatizare”, „Resume”, „Exporturi”.
- Cod existent:
  - Progressive artefacte în `LidarOdometryParams`: `progressive_*` (nume fișiere, m_poses, index_poses, point_sizes) – folosite în `compute_step_2(...)` și `save_result(...)`.
  - `session.json`, `index_poses.json`, `point_sizes_per_chunk.json` – scrise în `save_result(...)` și la final în streaming.
- Integrări propuse (TODO):
  - `state.json` + AtomicWriter: la finalul fiecărui chunk (în `compute_step_2(...)` când `progressive_save`, și în `save_streaming_chunk(...)`).
  - Resume detector: înainte de rulare (în wrapper GUI/CLI) folosește `load_worker_data_from_results(...)` pentru a reconstrui progresul.

2) Profiluri viteza/calitate, bugete, rafinare selectivă
- Plan: secțiunea „Viteză vs. Calitate”.
- Hook: buget per‑chunk în `compute_step_2(...)` (după optimizarea LO a chunk‑ului curent și înainte de salvare) – decide dacă aplică un mic „boost” (ex: +1 iterație, epsilon*0.5) în limitele `max_refinement_time_ms_per_chunk`.
- Parametri: adăugare în `LidarOdometryParams` + TOML; gating logic în bucla din `compute_step_2(...)`.

3) Defazaj la referință / obiecte dinamice
- Plan: „Mitigare obiecte dinamice”.
- Cod existent:
  - Ținte NDT se construiesc cu `update_rgd(...)`/`update_rgd_spherical_coordinates(...)` pe puncte „inițiale”/acumulate.
- Hook (MVP `ndt_exclude_recent_poses`):
  - La construcția țintei pentru chunk curent, exclude ultimele M poziții din fereastra rulantă (în secvența ce pregătește `buckets_indoor/outdoor` în `compute_step_2(...)`).
  - Alternativ, în `hessian_fun_*` (optimizatori) sări observațiile care lovesc voxeli asociați cu ultimele M poses.

4) Analiză holistică IMU (ZUPT, gaps, dinamici)
- Plan: secțiunea dedicată IMU.
- Cod existent: `calculate_trajectory(...)` și IMU încărcat în `load_data(...)`.
- Hook: funcție nouă `compute_imu_insights(imu_data)`: produce `imu_insights.json` și semnale ieftine (ZUPT, gaps, vibration_rms) înainte de `compute_step_1(...)`.

5) Exporturi: traiectorie îmbogățită, calitate, nori asamblați
- Plan: secțiunea „Exporturi” + „Traiectorie îmbogățită”.
- Cod existent: scriere per‑chunk `trajectory_lio_[c].csv` și LAZ cu sidecar CSV (intensity/timestamps/radii) în `save_result(...)` și `save_streaming_chunk(...)`.
- Hook:
  - Traiectorie îmbogățită: scriere CSV/JSON extra din `save_result(...)` (sau incremental per‑chunk când progressive); calcule scalari din `worker_data[i]` + IMU.
  - Calitate: `summary.json`, `per_chunk_metrics.csv` generate incremental în `compute_step_2(...)`.
  - Nor asamblat: agregare incrementală cu voxelizare la prag de segment; scriere în `exports.pointcloud` dacă bugetul permite.

6) Detectare/etichetare obiecte în mișcare
- Plan: secțiunea „Detectare și clasificare...”.
- Hook minim: după optimizarea chunk‑ului, construiește voxel grid low‑res al țintei (fără ultimele M poses), calculează scor rezidual pe punctele chunkului; dacă `filter_before_save`, aplică înainte de `exportLaz(...)` (în `save_result(...)`/`save_streaming_chunk(...)`).

7) Auto‑parametrizare (assist/offline/online)
- Plan: secțiunea „Auto‑parametrizare...”.
- Hook:
  - assist/offline: pilot scurt înainte de `compute_step_2(...)`, scrie `params_suggestions.json` și re‑rulează cu recomandările.
  - online: mic engine în bucla `compute_step_2(...)` care aplică modificări mici cu cooldown; jurnal în `params_timeline.json`.

8) Reprocesare segment, splice
- Plan: secțiunea „Reprocesare segment...”.
- Cod existent util: `load_worker_data_from_results(...)`, `load_index_poses(...)`, `load_point_sizes(...)`, scriere atomică în `save_result(...)`.
- Hook: `reprocess_segment(select, mode, overrides)` la nivel de app; folosește funcțiile de încărcare pentru a tăia/înlocui intervale și a re‑scrie artefactele.

9) TOML/JSON și GUI/CLI
- TOML: extindere `TomlIO` pentru noile grupuri `[performance]`, `[motion_segmentation]`, `[exports.*]`, `[auto_tune]`, `[reprocess]`.
- GUI/CLI: toggles pentru profil/bugete/exporturi; status per‑chunk (ok/refined/skipped), plus „Reprocesare segment” dialog.

Rezumat integrări rapide (low‑risk, Faza 1)
- Progressive resume: `state.json` + atomic writer la fiecare chunk [compute_step_2/save_streaming_chunk].
- Profiluri + buget local: +1 iterație/epsilon*0.5 când low‑confidence sub buget [compute_step_2].
- Defazaj MVP: exclude ultimele M poses din țintă [compute_step_2 buckets].
- Export traiectorie îmbogățită: CSV separat cu scalari de bază [save_result].
