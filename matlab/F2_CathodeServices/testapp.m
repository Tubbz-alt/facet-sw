mode=1; % memory leak when mode=1, not when mode=2
app=test;
nx=656; ny=492;
xax=linspace(0,1,nx); yax=linspace(0,1,ny);
if mode==1
  h=app.UIAxes;
else
  h=axes;
end
firstcall=true;
while 1
  img=rand(nx,ny);
  pos1=rand(1,2); pos2=rand(1,2);
  if firstcall
    cla(h);
    axis(h,[0 1 0 1]),hold(h,'on')
    imagesc(h,img,'XData',xax,'YData',yax); xlabel(h,'X'); ylabel(h,'Y');
    axis(h,'image');
    firstcall=false;
  else
    delete(h.Children(1:end-1));
    h.Children(end).CData=img;
  end
  hold(h,'on');
  plot(h,pos1(1),pos1(2),'rx','MarkerSize',12,'LineWidth',1);
  plot(h,pos2(1),pos2(2),'k.','MarkerSize',20); 
  rectangle(h,'Position',[0.25,0.25,0.5,0.5],'Curvature',1,'EdgeColor','k','LineWidth',3,'LineStyle','-');
  plot(h,0.5,0.5,'k+','MarkerSize',20,'LineWidth',3);
  rectangle(h,'Position',[0.3,0.3,0.35,0.35],'EdgeColor',[0.8500 0.3250 0.0980],'LineWidth',2,'LineStyle','--');
  rectangle(h,'Position',[0.2,0.2,0.6,0.6],'EdgeColor',[0.8500 0.3250 0.0980],'LineWidth',2,'LineStyle','--');
  drawnow limitrate
end