classdef Task_landing_altitude < Task
    properties
        feedbackGain = 0.5; % Guadagno di retroazione per l'altitudine
        targetAltitude = 0.0; % Altitudine desiderata (tocco del fondale)
    end
    
    methods
        % Aggiorna il riferimento desiderato xdotbar (scalare)
        function updateReference(obj, robot)
            % Errore di altitudine: negativo se siamo sopra il target, quindi scendiamo.
            err = obj.targetAltitude - robot.altitude;
            obj.xdotbar = obj.feedbackGain * err;
        end
        
        % Aggiorna la Jacobiana del task (1x13 per una grandezza scalare)
        function updateJacobian(obj, robot)
            % L'altitudine dipende dalla velocità verticale del veicolo.
            % Prendiamo la componente verticale (asse Z del mondo o del veicolo)
            % Supponendo che la jacobiana del veicolo mappi le velocità v_nu 
            % e che la componente verticale sia legata alla terza riga della rotazione o all'asse z:
            R = robot.wTv(1:3,1:3);
            % La Jacobiana ha dimensione 1 x 13: 
            % [7 zeri per il manipolatore, 3 per la velocità lineare del veicolo (direzione z), 3 per la velocità angolare]
            obj.J = [zeros(1, 7), R(3, :), zeros(1, 3)];
        end
        
        % Aggiorna la matrice di attivazione A (scalare per un task scalare)
        function updateActivation(obj, robot)
            % Il task di landing è sempre attivo.
            obj.A = 1.0;
        end
    end
end