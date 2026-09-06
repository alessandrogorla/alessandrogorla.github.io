clear
close all
clc

scelta_masses = questdlg('Come vuoi trattare le masse del robot durante l''ottimizzazione?', ...
                          'Scelta modalità ottimizzazione', ...
                          'Voglio prima stimarle', 'Assumo di conoscerle', 'Assumo di conoscerle');
        
switch scelta_masses
        
    case 'Voglio prima stimarle'
        
        fprintf('Le masse verranno stimate.\n');

        scelta_tipologia = questdlg('Come desideri ottimizzare i 21 coefficienti PID del robot Panda?', ...
                          'Scelta modalità ottimizzazione', ...
                          'Tutti insieme', 'A cascata (per braccio)', 'Tutti insieme');
        
        switch scelta_tipologia
        
            case 'Tutti insieme'

                fprintf('I 21 coefficienti verranno ottimizzati tutti insieme.\n');

                scriptPath = fullfile(pwd, 'Specific codes', '03 - Mass & Cascade', 'BOMain_masses_total.m');

                if exist(scriptPath, 'file') == 2
                    run(scriptPath);
                else
                    error('Lo script in questione non è stato trovato.');
                end

            case 'A cascata (per braccio)'

                fprintf('I 21 parametri saranno ottimizzati braccio per braccio.\n');

                scriptPath = fullfile(pwd, 'Specific codes', '03 - Mass & Cascade', 'BOMain_masses_total.m');

                if exist(scriptPath, 'file') == 2
                    run(scriptPath);
                else
                    error('Lo script in questione non è stato trovato.');
                end

        end

    case 'Assumo di conoscerle'
        
        fprintf('Si assume di conoscere perfettamente le masse del robot.\n');

        scelta_tipologia = questdlg('Come desideri ottimizzare i 21 coefficienti PID del robot Panda?', ...
                          'Scelta modalità ottimizzazione', ...
                          'Tutti insieme', 'A cascata (per braccio)', 'Tutti insieme');
        
        switch scelta_tipologia
        
            case 'Tutti insieme'

                fprintf('I 21 coefficienti verranno ottimizzati tutti insieme.\n');

                scelta_CF = questdlg('Che cost function vuoi utilizzare per ottimizzare i parametri del PID?', ...
                          'Scelta modalità ottimizzazione', ...
                          'Cost function 1', 'Cost function 2', 'Cost function 1');
        
                switch scelta_CF
        
                    case 'Cost function 1'
        
                        fprintf('Per l''ottimizzazione verrà usata la prima cost function.\n');
        
                        scriptPath = fullfile(pwd, 'Specific codes', '01 - Cost functions', 'Cost function 1', 'BOMain_CF1.m');
                
                        if exist(scriptPath, 'file') == 2
                            run(scriptPath);
                        else
                            error('Lo script in questione non è stato trovato.');
                        end
        
                    case 'Cost function 2'
        
                        fprintf('Per l''ottimizzazione verrà usata la seconda cost function.\n');
        
                        scriptPath = fullfile(pwd, 'Specific codes', '01 - Cost functions', 'Cost function 2', 'BOMain_CF2.m');
                
                        if exist(scriptPath, 'file') == 2
                            run(scriptPath);
                        else
                            error('Lo script in questione non è stato trovato.');
                        end
                        
                end

            case 'A cascata (per braccio)'

                fprintf('I 21 parametri saranno ottimizzati braccio per braccio.\n');

                scriptPath = fullfile(pwd, 'Specific codes', '02 - Cascade', 'BOMain_cascade.m');

                if exist(scriptPath, 'file') == 2
                    run(scriptPath);
                else
                    error('Lo script in questione non è stato trovato.');
                end
        end

end

disp(''); 
fprintf('\n');