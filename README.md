# hunav_gz_plugin

This repository provides the **`hunav_gz_plugin`**, specifically designed as the **Arena HuNav Plugin for Gazebo Garden (`gz-sim`)**.

It integrates the **HuNavSim pedestrian simulation** with the **Arena-Rosnav navigation framework**, enabling pedestrian-agent behavior in dynamic simulation environments.
(The original wrapper for Gazebo Fortress was used for the Migration to Gazebo Garden and for the Modifcation for Arena Rosnav: https://github.com/robotics-upo/hunav_gazebo_fortress_wrapper)

---

## 🔗 Related Projects

- **Arena-Rosnav**: [https://github.com/Arena-Rosnav](https://github.com/Arena-Rosnav)  
- **HuNavSim** (pedestrian simulation): [https://github.com/robotics-upo/hunav_sim](https://github.com/robotics-upo/hunav_sim)  
- **Arena fork for HuNavSim**: [https://github.com/voshch/hunav_sim](https://github.com/voshch/hunav_sim)

---

## 🧠 Plugin Description

The plugin connects to the **`HunavManager`** of Arena-Rosnav, which functions as a dedicated entity manager for HuNavSim.  
The HunavManager replaces the originally provided `WorldManager` from the HuNavSim project and was tailored specifically for Arena-Rosnav compatibility and modular simulation setups.

---

## 🚶 Actor Simulation & Obstacle Avoidance

Gazebo actors lack physical interactions by default. Therefore, this plugin is **responsible for their full kinematic control**, including:

- Agent path updates
- Navigation goal processing
- Obstacle avoidance

To enable avoidance of static obstacles such as **walls**, additional logic and infrastructure was implemented.

---

## 📡 Published Topics

Agent state information is published on the following topics:

- `/people`
- `/human_agents`

---

## 🧱 Wall Integration Support

An additional **custom service and message definition** was introduced in the Arena fork of HuNavSim ([voshch/hunav_sim](https://github.com/voshch/hunav_sim)) to expose wall segment data.  
This allows the plugin to receive detailed wall geometry and integrate it into the agent avoidance logic robustly.

---

## Still in Progress 

---

## 📜 License

MIT License – see [LICENSE](./LICENSE) for details.
