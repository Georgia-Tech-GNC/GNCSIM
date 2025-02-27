function c = tprintf(varargin)
% Terminal Print
% c = tprintf( FigureName, Text );
% 
% Example:
%     tprintf( 'MyMessages', 'Hello world in 2nd terminal!' );
% 
% sends text 'Hello world in 2nd terminal!' to a window with the name MyMessages.
% 
% Clearing the window: to clear the window, call without the 2nd argument:
% 
%     tprintf( FigureName );
% 
% Create window if it does not exist:
% 
%     tprintf( FigureName , '' );
%   
% 
% Requires external function findjobj(), provided by Yair Altman online from Matlab File Exchange
% 
% Peter Adany 2014
% 

    %% Inputs:
    figureName = varargin{1};
    if nargin<2
        strNewText = '';
        doAllClear = 1;
    else
        strNewText = varargin{2};
        doAllClear = 0;
    end
    
    %% Establish target window:
    fH = findobj( allchild(0), 'flat', 'Name', figureName ); % find the figure if it exists  (thanks Jan Simon for the suggestion)
    if isempty(fH)
        windowExists = 0;
    else
        windowExists = 1;
    end
    windowEnabled = ~( nargin==1 && ~windowExists ); % this flag indicates we don't have and do not want to create a new window
    if windowEnabled
        if ~windowExists
            % obtain figure with non-integer number (thanks Jan Simon for the suggestion):
            fH = figure('IntegerHandle','off'); 
            set(fH,'Name',figureName,'NumberTitle','off');
            % create a text box in the figure:
            sH = uicontrol(fH,'Style','edit','Max',2,...
                'FontName','Monospaced','FontSize',10,...
                'String','',...
                'Units','normalized','Position',[0 0 1 1],'BackgroundColor',[1 1 1],'ForegroundColor',[0 0 0],'HorizontalAlignment','Left');
            %  BEGIN THIS PART Thanks to Yair Altman and others on Matlab forums.
            %  enable horizontal scrolling 
             jEdit = findjobj(sH);
            ijEdit = length(jEdit); % picking the last one as default (modified by PEA)
            jEditbox = jEdit(ijEdit).getViewport().getComponent(0);
            jEditbox.setWrapping(false);                % turn off word-wrapping
            jEditbox.setEditable(false);                % non-editable
            set(jEdit(ijEdit),'HorizontalScrollBarPolicy',30);  % HORIZONTAL_SCROLLBAR_AS_NEEDED
            % maintain horizontal scrollbar policy which reverts back on component resize :
            jhEdit = handle(jEdit(ijEdit),'CallbackProperties');
            set(jhEdit, 'ComponentResizedCallback', 'set(gcbo,''HorizontalScrollBarPolicy'',30);hE=gcbo;chE=hE.getComponent(0).getComponent(0);chE.setCaretPosition(0);hE.repaint;' ); % (modified by PEA)
            %  END THIS PART Thanks to Yair Altman and others on Matlab forums.
        else
            sH = findobj(fH,'-depth',1,'Type','uicontrol');
        end
    end
    
    %% Update with new text:
    if windowEnabled
        % set up blank lines to pad out the bottom of the scroll view and resemble the command window:
        padEndNumLines = 2;
        padEndLabels={'  ','>>'};
        % read current contents:
        tStr = get(sH,'String');
        nLin = size(tStr,1);
        % strip trailing blank lines:
        tStr = tStr(1:nLin-padEndNumLines,:);
        % append new line of text and pad end lines:
        if isempty(tStr)
            tStr = strNewText;
        else
            tStr = char(tStr,strNewText);
        end
        for iPad = 1:padEndNumLines
            tStr = char(tStr, sprintf(padEndLabels{max(iPad-padEndNumLines+2,1)}) ); % lazy method to use first index for all-but-final pad lines and second index for the final pad line
        end
        if ~windowExists && isempty(strNewText)
            tStr = tStr(2:end,:);       % eliminate the new line entry on window creation caused by empty string in 2nd argin
        end
        if doAllClear
            tStr = '';
            nLin = size(tStr,1);
        end
        % update data field:
        set(sH,'String',tStr);
        % knock the scroll & caret back to preferred bottom-left positions:
           jEdit = findjobj(sH);
          ijEdit = length(jEdit);
        jEditbox = jEdit(ijEdit).getComponent(0).getComponent(0);
        jEditbox.setCaretPosition(0);
          jhEdit = handle(jEdit(ijEdit),'CallbackProperties');
        jVScroll = jhEdit.getVerticalScrollBar;
        jVScroll.setValue(jVScroll.getMaximum);
        if(0)
            jhEdit.repaint; % refresh
        end
    end
    
    %% Return something similar to character count as with fprintf()
    if windowEnabled
        c = max( size(tStr) - [ padEndNumLines 0] , 0);
    else
        c = [0 0];
    end
    
end