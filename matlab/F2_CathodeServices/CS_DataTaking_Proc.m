% process CS_DataTaking.m data
close all

t=(1:ndat)./rate./60;

figure
plot(xv(shut==1),yv(shut==1),'.',xv(shut==0),yv(shut==0),'.')
xlabel('X [mm]'); ylabel('Y [mm]');
grid

figure
plot(t(shut==0),lener(shut==0),'.');
xlabel('Time [min]'); ylabel('Laser Energy (joulemeter) [\mum]');
mener=mean(lener(shut==0)); stdener=std(lener(shut==0)); ranener=range(lener(shut==0));
legend({sprintf('Laser Energy = %.2f +/- %.2f range= %.2f uJ',mener,stdener,ranener)})
grid

figure
cut=ss_y<500 & shut==0;
plot(t(cut),ss_x(cut),'.',t(cut),ss_y(cut),'.')
mx=mean(ss_x(cut)); stdx=std(ss_x(cut));
my=mean(ss_y(cut)); stdy=std(ss_y(cut)); 
xlabel('Time [min]'); ylabel('FWHM Laser Spot Size [\mum]');
legend({sprintf('X = %.2f +/- %.2f \\mum',mx,stdx),sprintf('Y= %.2f +/- %.2f \\mum',my,stdy)});
grid

figure
cut=ss_y<500 & shut==0;
fluence=1e6*lener(cut)./(ss_x(cut).*ss_y(cut));
plot(t(cut),fluence,'.')
xlabel('Time [min]'); ylabel('Laser fluence [ \muJ / mm^2]');
legend({sprintf('Laser Fluence = %.2f +/- %.2f range= %.2f \\muJ/mm^2',mean(fluence),std(fluence),range(fluence))})
grid