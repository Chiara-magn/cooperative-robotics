classdef Task_min_altitude < Task
    properties
        feedbackGain = 2.0; % Guadagno di retroazione per l'altitudine
    end
    
    methods
        % Aggiorna il riferimento desiderato xdotbar (scalare)
        function updateReference(obj, robot)
            current_altitude = robot.altitude;  
            threshold_altitude = 2.0; % Soglia di sicurezza minima dal fondo (metri)
            
            % Se siamo sotto la soglia, generiamo una velocità positiva per salire
            if current_altitude < threshold_altitude
                error = threshold_altitude - current_altitude;
                obj.xdotbar = obj.feedbackGain * error;
            else
                obj.xdotbar = 0;
            end
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
            current_altitude = robot.altitude;
            threshold_altitude = 2.0;
            transition_band = 1.0; % Banda di transizione per attivare gradualmente il task
            
            if current_altitude < threshold_altitude
                obj.A = 1.0; % Completamente attivo se siamo sotto la soglia
            elseif current_altitude < (threshold_altitude + transition_band)
                % Attivazione graduale (smooth transition tra 0 e 1)
                obj.A = 1 - (current_altitude - threshold_altitude) / transition_band;
            else
                obj.A = 0.0; % Disattivo se siamo sufficientemente lontani dal fondo
            end
        end
    end
end