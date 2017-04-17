%% Randomly generated graphs
clear variables ;
close all ;

p = [5,10,20] ; % sigma^2_max
tt = 0:99 ;

%% Box plots
plotData = input('Do you wish to view and print box plot data? ','s') ;
if (strcmpi(plotData,'y'))
    fn = 'times' ;
    for t = 0:16 % trial number
        close all ;
        for k = 1:numel(p)
            fileDir = sprintf('logs/maxVar_%d/pThresh_0.65',p(k)) ;
            data0 = load(sprintf('%s/path_costs%d.txt',fileDir,t)) ;
            data = zeros(size(data0)) ;
            data(:,1) = data0(:,1) ;
            data(:,2:3) = data0(:,3:4) ;
            data(:,4) = data0(:,2) ;
            data(:,5) = data0(:,5) ;
            diffCost = zeros(size(data)) ;

            for i = 1:size(data,1)
                for j = 1:size(data,2)
                    diffCost(i,j) = (data(i,j) - data(i,end))./data(i,end)*100 ;
                    if diffCost(i,j) > 1e3
                        diffCost(i,j) = NaN ;
                    elseif diffCost(i,j) < 0
                        fprintf('Entry (%d,%d): %f\n',i,j,diffCost(i,j)) ;
                    end
                end
            end

            figure(k), clf ;
            hold on
            pt = ['Comparison of Path Costs for Graph with $\sigma_{\textit{max}}^2=',sprintf('%i',p(k)),'$'] ;
            title(pt,'interpreter','latex')
            ylabel('Path cost percentage above optimal','interpreter','latex')
            hb = boxplot(diffCost(:,1:4)) ;
            set(gca,'xtick',[1,2,3,4],'xticklabel',{'RAGS  ','Na\"{i}ve A*  ','Sampled A*  ','Greedy  '},'ticklabelinterpreter','latex') ;
            y_lim = get(gca,'ylim') ;
            %     set(gca,'ylim',[-5,y_lim(2)]) ;
            set(gca,'ylim',[0,y_lim(2)],'fontname',fn) ;
            th = rotateticklabel(gca,45) ;
            set(th,'fontsize',get(gca,'fontsize'),'interpreter','latex') ;
            set(gca,'xticklabel',{''}) ;
            set(gcf,'position',[(150+350*(k-1)) 385 330 420])
            pos = get(gca,'position') ;
            pos(2) = 0.1 ;
            set(gca,'position',pos)
            if k < 3
                set(k,'paperposition',[-0.1 0.1 4.5833 5.9333])
                set(k,'papersize',[4.1 5.8])
            else
                set(k,'paperposition',[-0.04 0.1 4.5833 5.9333])
                set(k,'papersize',[4.15 5.8])
            end
        end
        drawnow ;
        pause ;

        s = sprintf('graph%i_maxVX',t) ;
        sp = input(sprintf('Do you wish to print these figures to pdf? (Figures will be saved as ''%s'') ',s),'s') ;
        switch lower(sp)
            case 'y'
                for k = 1:numel(p)
                    print(k,sprintf('plots/p_thresh 0.60/graph%i_maxV%i',t,p(k)),'-dpdf','-r0') ;
                end
        end
    end
end

%% Path cost statistics
costStats = input('Do you wish to calculate path cost statistics? ','s') ;
if (strcmpi(costStats,'y'))
    quartiles = zeros(numel(tt),16) ;
    for k = 1:numel(p)
        for t = tt % trial number
            fileDir = sprintf('logs/maxVar_%d/pThresh_0.60',p(k)) ;
            data0 = load(sprintf('%s/path_costs%d.txt',fileDir,t)) ;
            data = zeros(size(data0)) ;
            data(:,1) = data0(:,1) ;
            data(:,2:3) = data0(:,3:4) ;
            data(:,4) = data0(:,2) ;
            data(:,5) = data0(:,5) ;
            diffCost = zeros(size(data)) ;
            
            for i = 1:size(data,1)
                for j = 1:size(data,2)
                    diffCost(i,j) = (data(i,j) - data(i,end))./data(i,end)*100 ;
                    if diffCost(i,j) > 1e3
                        diffCost(i,j) = NaN ;
                    elseif diffCost(i,j) < 0
                        fprintf('Entry (%d,%d): %f\n',i,j,diffCost(i,j)) ;
                    end
                end
            end
            
            q = quantile(diffCost,3) ;
            for i = 1:4
                quartiles(t+1,(i-1)*4+(1:3)) = q(:,i)' ;
                quartiles(t+1,(i-1)*4+4) = max(diffCost(:,i)) ;
            end
        end
        fprintf('Max variance: %.2f\n',p(k)) ;
        fprintf('Planning algorithm\t1st\t2nd\t3rd\tWorst\n') ;
        fprintf('RAGS\t\t\t%.2f\t%.2f\t%.2f\t%.2f\n',mean(quartiles(:,1)),mean(quartiles(:,2)),mean(quartiles(:,3)),max(quartiles(:,4))) ;
        fprintf('Naive A*\t\t%.2f\t%.2f\t%.2f\t%.2f\n',mean(quartiles(:,5)),mean(quartiles(:,6)),mean(quartiles(:,7)),max(quartiles(:,8))) ;
        fprintf('Sampled A*\t\t%.2f\t%.2f\t%.2f\t%.2f\n',mean(quartiles(:,9)),mean(quartiles(:,10)),mean(quartiles(:,11)),max(quartiles(:,12))) ;
        fprintf('Greedy\t\t\t%.2f\t%.2f\t%.2f\t%.2f\n',mean(quartiles(:,13)),mean(quartiles(:,14)),mean(quartiles(:,15)),max(quartiles(:,16))) ;
    end
end

%% Computation time
compTime = input('Do you wish to calculate computation time statistics? ','s') ;
if (strcmpi(compTime,'y'))
    n = 100 ;
    cTime = zeros(numel(tt)*n,12) ;
    for k = 1:numel(p)
        for t = tt % trial number
            fileDir = sprintf('logs/maxVar_%d/pThresh_0.50',p(k)) ;
            data = load(sprintf('%s/computation_times%d.txt',fileDir,t)) ;
            cTime(t*n+1:t*n+n,(k-1)*4+(1:4)) = data ;
        end
    end
    meanCTime = zeros(numel(p),4) ;
    stdCTime = zeros(numel(p),4) ;
    for i = 1:numel(p)
        [meanCTime(i,:),stdCTime(i,:)] = normfit(cTime(:,(i-1)*4+(1:4))) ;
    end
    fprintf('Mean computation time:\n') ;
    fprintf('    RAGS      Greedy    Naive A*  Sampled A*\n') ;
    disp(meanCTime) ;
    fprintf('Standard deviation:\n') ;
    fprintf('    RAGS      Greedy    Naive A*  Sampled A*\n') ;
    disp(stdCTime) ;
end

%% All path cost statistics
costStats = input('Do you wish to plot path cost statistics? ','s') ;
if (strcmpi(costStats,'y'))
    mV = input('Max variance to plot: ') ;
    pT = 0.5:0.05:0.75 ;
    pQ = zeros(3,numel(pT),4) ; % (quartiles,p_thresh,alg)
    for k = 1:numel(pT)
        quartiles = zeros(numel(tt),16) ;
        for t = tt % trial number
            fileDir = sprintf('logs/maxVar_%d/pThresh_%.2f',mV,pT(k)) ;
            data0 = load(sprintf('%s/path_costs%d.txt',fileDir,t)) ;
            data = zeros(size(data0)) ;
            data(:,1) = data0(:,1) ;
            data(:,2:3) = data0(:,3:4) ;
            data(:,4) = data0(:,2) ;
            data(:,5) = data0(:,5) ;
            diffCost = zeros(size(data)) ;

            for i = 1:size(data,1)
                for j = 1:size(data,2)
                    diffCost(i,j) = (data(i,j) - data(i,end))./data(i,end)*100 ;
                    if diffCost(i,j) > 1e3
                        diffCost(i,j) = NaN ;
                    elseif diffCost(i,j) < 0
                        fprintf('Entry (%d,%d): %f\n',i,j,diffCost(i,j)) ;
                    end
                end
            end
            
            q = quantile(diffCost,3) ;
            for i = 1:4
                quartiles(t+1,(i-1)*4+(1:3)) = q(:,i)' ;
                quartiles(t+1,(i-1)*4+4) = max(diffCost(:,i)) ;
            end
        end
        for i = 1:4
            pQ(:,k,i) = mean(quartiles(:,(i-1)*4+(1:3)))' ;
        end
    end
    figure
    c = get(gca,'colororder') ;
    fs = 12 ;
    fn = 'times' ;
    hold on
    pc = zeros(4,1) ;
    pe = zeros(4,1) ;
    pp = zeros(4,1) ;
    patch_x = [pT,fliplr(pT)] ;
    oF = 0.005 ;
    offset = -(1.5*oF):oF:(1.5*oF) ;
    leg_string = {'RAGS','Na\"{i}ve A*','Sampled A*','Greedy'} ;
    lw = [3,2,3,2] ;
    for i = 1:4
        patch_PQy = [pQ(1,:,i),fliplr(pQ(3,:,i))] ;
        pc(i) = plot(pT+offset(i)*ones(size(pT)),pQ(2,:,i),'-','color',c(i,:),'linewidth',lw(i)) ;
%         pp(i) = patch(patch_x,patch_PQy,c(i,:),'facealpha',0.2,'linestyle','none') ;
        pe(i) = errorbar(pT+offset(i)*ones(size(pT)),pQ(2,:,i),pQ(2,:,i)-pQ(1,:,i),pQ(3,:,i)-pQ(2,:,i),'color',c(i,:),'linestyle','none') ;
    end
    axis tight ;
    legend(pc,leg_string,'interpreter','latex','location','northwest','fontsize',fs) ;
    set(gca,'fontsize',fs,'fontname',fn)
    ylim = get(gca,'ylim') ;
    ylim = [floor(ylim(1)),ceil(ylim(2))] ;
    set(gca,'ylim',ylim)
    pt = ['Comparison of Path Costs for Graphs with $\sigma_{\textit{max}}^2=',sprintf('%i',mV),'$'] ;
    ht = title(pt,'interpreter','latex','fontsize',fs);
    ylabel('Path cost percentage above optimal','interpreter','latex','fontsize',fs)
    xlabel('Domination threshold $p_{\mathit{thresh}}$','interpreter','latex','fontsize',fs) ;
    s = sprintf('costCompareAll_maxV%d.pdf',mV) ;
    sp = input(sprintf('Do you wish to print this figure to pdf? (Figure will be saved as ''%s'') ',s),'s') ;
    switch lower(sp)
        case 'y'
%             export_fig(gcf,sprintf('plots/%s',s),'-trans') ;
            set(gcf,'paperposition',[-0.55 -0.2 8 6])
            set(gcf,'papersize',[6.75,5.65])
            print(gcf,sprintf('plots/%s',s),'-dpdf','-r0') ;
    end
end

%% All computation time statistics
compTime = input('Do you wish to plot computation time statistics? ','s') ;
if (strcmpi(compTime,'y'))
    mV = input('Max variance to plot: ') ;
    pT = 0.5:0.05:0.75 ;
    pCmean = zeros(numel(pT),4) ; % (p_thresh,alg)
    pCU = zeros(numel(pT),4) ; % (p_thresh,alg)
    pCL = zeros(numel(pT),4) ; % (p_thresh,alg)
    pCstd = zeros(numel(pT),4) ; % (p_thresh,alg)
    n = 100 ;
    cTime = zeros(numel(tt)*n,4) ;
    for k = 1:numel(pT)
        for t = tt % trial number
            fileDir = sprintf('logs/maxVar_%d/pThresh_%.2f',mV,pT(k)) ;
            data = load(sprintf('%s/computation_times%d.txt',fileDir,t)) ;
            cTime(t*n+(1:n),:) = [data(:,1),data(:,3),data(:,4),data(:,2)] ;
        end
        [pCmean(k,:),pCstd(k,:),ciCTime] = normfit(cTime) ;
        pCL(k,:) = ciCTime(1,:) ;
        pCU(k,:) = ciCTime(2,:) ;
        
    end
    figure
    c = get(gca,'colororder') ;
    fs = 12 ;
    fn = 'times' ;
    hold on
    pct = zeros(4,1) ;
    pect = zeros(4,1) ;
    pp = [1,2,4] ;
    leg_string = {'RAGS','Na\"{i}ve A*','Sampled A*','Greedy'} ;
    lw = [3,2,3,2] ;
    for i = 1:numel(pp)
        if (i == 2)
            [hax,pct(pp(i)),pct(pp(i)+1)] = plotyy(pT,pCmean(:,pp(i)),pT,pCmean(:,3)) ;
            pect(pp(i)+1) = errorbar(pT,pCmean(:,pp(i)+1),pCmean(:,pp(i)+1)-pCL(:,pp(i)+1),pCU(:,pp(i)+1)-pCmean(:,pp(i)+1),'color',c(pp(i)+1,:),'linestyle','none') ;
            set(pct(pp(i)),'linestyle','-','color',c(pp(i),:),'linewidth',lw(pp(i))) ;
            set(pct(pp(i)+1),'linestyle','-','color',c(pp(i)+1,:),'linewidth',lw(pp(i)+1)) ;
        else
            pct(pp(i)) = plot(pT,pCmean(:,pp(i)),'-','color',c(pp(i),:),'linewidth',lw(pp(i))) ;
        end
        pect(pp(i)) = errorbar(pT,pCmean(:,pp(i)),pCmean(:,pp(i))-pCL(:,pp(i)),pCU(:,pp(i))-pCmean(:,pp(i)),'color',c(pp(i),:),'linestyle','none') ;
    end
    if mV == 5
        set(hax(1),'xlim',[0.4975 0.7525],'ylim',[0 .1],'ycolor','k','ytick',0:0.02:0.1) ;
        set(hax(2),'xlim',[0.4975 0.7525],'ylim',[0 5],'ycolor','k','ytick',0:5) ;
    elseif mV == 10
        set(hax(1),'xlim',[0.4975 0.7525],'ylim',[0 1],'ycolor','k','ytick',0:0.2:1) ;
        set(hax(2),'xlim',[0.4975 0.7525],'ylim',[0 64],'ycolor','k','ytick',0:20:60) ;
    end
    legend(pct,leg_string,'interpreter','latex','location','northwest','fontsize',fs) ;
    set(hax(1),'fontsize',fs,'fontname',fn)
    set(hax(2),'fontsize',fs,'fontname',fn)
    set(pect(3),'parent',hax(2)) ;
    pt = ['Comparison of Planning Times for Graphs with $\sigma_{\textit{max}}^2=',sprintf('%i',mV),'$'] ;
    ht = title(pt,'interpreter','latex','fontsize',fs);
    ylabel(hax(1),'Computation time (s)','interpreter','latex','fontsize',fs)
    ylabel(hax(2),'Graph samples','interpreter','latex','fontsize',fs)
    xlabel('Domination threshold $p_{\mathit{thresh}}$','interpreter','latex','fontsize',fs) ;
    s = sprintf('timeCompareAll_maxV%d.pdf',mV) ;
    sp = input(sprintf('Do you wish to print this figure to pdf? (Figure will be saved as ''%s'') ',s),'s') ;
    switch lower(sp)
        case 'y'
%             export_fig(gcf,sprintf('plots/%s',s),'-trans') ;
            set(gcf,'paperposition',[-0.5 -0.2 8 6])
            set(gcf,'papersize',[7.2,5.65])
            print(gcf,sprintf('plots/%s',s),'-dpdf','-r0') ;
    end
end

%% Plot graph and ND set
close all ;
mV = 20 ;
t = 0 ;
fileDir = sprintf('logs/maxVar_%d/pThresh_0.70',mV) ;
ND_set = load(sprintf('%s/ND_set%d.txt',fileDir,t)) ;
RAGS = load(sprintf('%s/RAGS_path%d.txt',fileDir,t)) ;
edges = load(sprintf('config_files/maxVar_%d/edges%d.txt',mV,t)) ;
verts = load(sprintf('config_files/maxVar_%d/vertices%d.txt',mV,t)) ;
maxV = max(edges(:,4)) ;
g = input('Do you wish to plot graph? ','s') ;
if (strcmpi(g,'y'))
    figure ;
    hold on ;
    axis equal ;
    pe = zeros(size(edges,1),1) ;
    cm = colormap ;
    for i = 1:size(edges,1)
        cc = (edges(i,4)/maxV)*size(cm,1) ;
        fcm = max(1,floor(cc)) ;
        ccm = max(1,ceil(cc)) ;
        dc = 1-max(0,cc-fcm) ;
        c = dc*cm(fcm,:) + (1-dc)*cm(ccm,:) ;
        v = [verts(edges(i,1)+1,:);verts(edges(i,2)+1,:)] ;
        pe(i) = plot(v(:,1),v(:,2),'-','color',c) ;
    end
    axis tight ;
    set(gca,'visible','off') ;
    s = 'graph_full.pdf' ;
    sp = input(sprintf('Do you wish to print this figure to pdf? (Figure will be saved as ''%s'') ',s),'s') ;
    switch lower(sp)
        case 'y'
            export_fig(gcf,sprintf('plots/%s',s),'-trans') ;
    end
end

nd = input('Do you wish to plot nd-set? ','s') ;
if (strcmpi(nd,'y'))
    figure ;
    hold on ;
    axis equal ;
    pe = zeros(size(edges,1),1) ;
    cm = colormap ;
    for i = 1:size(edges,1)
        cc = (edges(i,4)/maxV)*size(cm,1) ;
        fcm = max(1,floor(cc)) ;
        ccm = max(1,ceil(cc)) ;
        dc = 1-max(0,cc-fcm) ;
        c = dc*cm(fcm,:) + (1-dc)*cm(ccm,:) ;
        v = [verts(edges(i,1)+1,:);verts(edges(i,2)+1,:)] ;
        pe(i) = plot(v(:,1),v(:,2),'-','color',c) ;
    end
    axis tight ;
    set(gca,'visible','off') ;
    pnd = zeros(size(ND_set,1)-1,1) ;
    for i = 1:size(ND_set,1)-1
        if (ND_set(i,1) == 100 && ND_set(i,2) == 100)
            continue ;
        else
            pnd(i) = plot(ND_set(i:i+1,1),ND_set(i:i+1,2),'r-','linewidth',2) ;
        end
    end
    prags = zeros(size(RAGS,1)-1,1) ;
    for i = 1:size(RAGS,1)-1
        prags(i) = plot(RAGS(i:i+1,1),RAGS(i:i+1,2),'k-','linewidth',2) ;
    end
    s = 'graph_full_nd_set.pdf' ;
    sp = input(sprintf('Do you wish to print this figure to pdf? (Figure will be saved as ''%s'') ',s),'s') ;
    switch lower(sp)
        case 'y'
            export_fig(gcf,sprintf('plots/%s',s),'-trans') ;
    end
end