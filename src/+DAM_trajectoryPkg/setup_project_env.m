function setup_project_env()


        % FOR ME
    % CasADi
    addpath('C:\Users\danil\OneDrive\Documents\GitHub\GNCSIM\src\casadi-3.7.2-windows64-matlab2018b')
    import casadi.*
    
    % acados
    addpath(genpath('C:\dev\acados\interfaces\acados_matlab_octave'))
    setenv('ACADOS_INSTALL_DIR','C:\dev\acados')
    
    % MinGW / CMake paths
    mingwBin = 'C:\ProgramData\MATLAB\SupportPackages\R2024a\3P.instrset\mingw_w64.instrset\bin';
    cmakeBin = 'C:\Program Files\CMake\bin';
    
    setenv('PATH', [mingwBin ';' cmakeBin ';' getenv('PATH')]);
    
    % force compiler tools
    setenv('CC',  fullfile(mingwBin,'gcc.exe'));
    setenv('CXX', fullfile(mingwBin,'g++.exe'));
    setenv('MAKE', fullfile(mingwBin,'mingw32-make.exe'));

    % repo_root = fileparts(mfilename('fullpath'));
    % 
    % % CasADi
    % casadi_dir = 'C:\dev\casadi-3.7.2-windows64-matlab2018b';
    % if ~exist(casadi_dir, 'dir')
    %     error('CasADi not found at %s', casadi_dir);
    % end
    % addpath(casadi_dir);
    % 
    % % acados
    % acados_dir = 'C:\dev\acados';
    % if ~exist(acados_dir, 'dir')
    %     error('acados not found at %s', acados_dir);
    % end
    % setenv('ACADOS_INSTALL_DIR', acados_dir);
    % addpath(genpath(fullfile(acados_dir, 'interfaces', 'acados_matlab_octave')));
    % 
    % % toolchain
    % mingwBin = 'C:\ProgramData\MATLAB\SupportPackages\R2024a\3P.instrset\mingw_w64.instrset\bin';
    % cmakeBin = 'C:\Program Files\CMake\bin';
    % 
    % if ~exist(mingwBin, 'dir')
    %     error('MATLAB MinGW support package not found.');
    % end
    % if ~exist(cmakeBin, 'dir')
    %     error('CMake not found.');
    % end
    % 
    % setenv('PATH', [mingwBin ';' cmakeBin ';' getenv('PATH')]);
    % setenv('CC', fullfile(mingwBin, 'gcc.exe'));
    % setenv('CXX', fullfile(mingwBin, 'g++.exe'));
    % setenv('MAKE', fullfile(mingwBin, 'mingw32-make.exe'));
    % 
    % import casadi.*
end