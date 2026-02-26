which rocketPkg.DatcomManager -all
mc = meta.class.fromName('rocketPkg.DatcomManager');
disp(mc.ContainingPackage.Name);
disp(which('rocketPkg.DatcomManager'));

% Show whether runDatcom exists in the currently-loaded definition
hasRun = any(strcmp({mc.MethodList.Name}, 'runDatcom'));
disp(hasRun);