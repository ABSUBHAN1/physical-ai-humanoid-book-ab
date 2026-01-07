import React from 'react';
import { translate } from '@docusaurus/Translate';

import styles from './SimulationEmbed.module.css';

const SimulationEmbed = ({ src, title, description }) => {
  return (
    <div className={styles.simulationContainer}>
      <div className={styles.simulationHeader}>
        <h3>{title || translate({ message: 'Robotics Simulation' })}</h3>
        {description && <p className={styles.description}>{description}</p>}
      </div>
      <div className={styles.simulationFrame}>
        <iframe
          src={src}
          title={title || translate({ message: 'Robotics Simulation' })}
          className={styles.iframe}
          allowFullScreen
          frameBorder="0"
        />
      </div>
      <div className={styles.simulationFooter}>
        <p>{translate({ message: 'Interact with the simulation to experiment with the concepts discussed in this chapter.' })}</p>
      </div>
    </div>
  );
};

export default SimulationEmbed;