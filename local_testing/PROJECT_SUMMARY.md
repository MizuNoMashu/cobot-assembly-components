# Franka Robot Controller - Stato Progetto

## Obiettivo

Ricostruire il progetto per il controllo di robot Franka Robotics tramite pylibfranka, seguendo gli step definiti nelle istruzioni Copilot.

## Stato attuale

- Step 1 in corso: definizione di pyproject.toml, documentazione e struttura progetto.
- Step 2 e Step 3 ancora da eseguire.

## Struttura prevista

```
robot_llm/
├── pyproject.toml
├── README.md
├── QUICKSTART.md
├── src/
│   └── franka_controller/
│       ├── __init__.py
│       ├── robot.py
│       ├── motion.py
│       ├── gripper.py
│       └── utils.py
├── examples/
│   ├── main.py
│   └── quick_examples.py
└── tests/
    └── test_robot.py
```
- Context manager per gestione risorse

### 🛡️ Sicurezza
- Validazione limiti articolari
- Configurazione collision behavior
- Controllo velocità parametrico
- Error handling robusto

### 📊 Controllo Avanzato
- Traiettorie minimum jerk per movimenti fluidi
- Controllo impedenza con stiffness/damping configurabili
- Gripper con controllo forza

### 📝 Documentazione Completa
- Docstrings dettagliate
- Esempi completi e funzionanti
- Guide di installazione e troubleshooting

### 🧪 Testabilità
- Test unitari per funzioni core
- Struttura pronta per mock testing
- Configurazione pytest completa

---

## 🎓 Prossimi Passi (Opzionali)

### Implementazioni Future:
- [ ] Cinematica inversa per controllo cartesiano
- [ ] Integrazione con librerie robotica (e.g., roboticstoolbox-python)
- [ ] Visualizzazione 3D dello stato robot
- [ ] Registrazione e replay di traiettorie
- [ ] ROS 2 integration
- [ ] Dashboard web per monitoraggio

### Miglioramenti Possibili:
- [ ] Pianificazione traiettorie con collision avoidance
- [ ] Force/torque feedback control
- [ ] Machine learning per grasp optimization
- [ ] Multi-robot coordination

---

## 📞 Supporto

Per problemi con:
- **pylibfranka**: Deve essere compilato da sorgenti - [guida ufficiale](https://github.com/frankarobotics/libfranka)
- **Questo progetto**: Vedi QUICKSTART.md e esempi
- **Robot Franka**: Consulta [Franka Desk Manual](https://www.franka.de/user-manuals)

## ⚠️ Problemi Comuni

### `pip install pylibfranka` non funziona
**Causa**: I package su PyPI (0.20.x) hanno metadata corrotti.

**Soluzione**: Installazione **OBBLIGATORIA da sorgenti** - vedi README.md sezione "Installazione pylibfranka"

---

## ⚠️ Note Importanti

1. **Prerequisito**: pylibfranka deve essere installato separatamente
2. **Sicurezza**: Testa sempre con speed_factor bassi inizialmente
3. **IP Robot**: Modifica l'IP negli esempi con quello del tuo robot
4. **Limiti**: Rispetta sempre i limiti articolari del robot

---

## 🎉 Progetto Completato!

Tutti e 3 gli step sono stati implementati con successo:
- ✅ Step 1: Struttura progetto e configurazione
- ✅ Step 2: Moduli richiamabili per controllo robot
- ✅ Step 3: Esempi completi e workflow

Il progetto è ora pronto per l'uso! 🚀
