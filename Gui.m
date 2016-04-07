function varargout = Gui(varargin)
% GUI MATLAB code for Gui.fig
%      GUI, by itself, creates a new GUI or raises the existing
%      singleton*.
%
%      H = GUI returns the handle to a new GUI or the handle to
%      the existing singleton*.
%
%      GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI.M with the given input arguments.
%
%      GUI('Property','Value',...) creates a new GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Gui

% Last Modified by GUIDE v2.5 06-Apr-2016 19:38:36

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Gui_OpeningFcn, ...
                   'gui_OutputFcn',  @Gui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Gui is made visible.
function Gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Gui (see VARARGIN)

% Choose default command line output for Gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function FilenameEdit_Callback(hObject, eventdata, handles)
% hObject    handle to FilenameEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FilenameEdit as text
%        str2double(get(hObject,'String')) returns contents of FilenameEdit as a double


% --- Executes during object creation, after setting all properties.
function FilenameEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FilenameEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Load.
function Load_Callback(hObject, eventdata, handles)
% hObject    handle to Load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    %Get file to open
    [filename, path] = uigetfile('*.mat','Preprocessed data file');
    if(filename~=0)
        %Say we are busy
        set(handles.figure1, 'pointer', 'watch');
        drawnow;
        
        filedat=load([path filename]);
        handles.filename = filedat.filename;
        handles.path = filedat.path;
        handles.CurrentRecording = filedat.CurrentRecording;
        
        %Update values in the GUI (nb movements, time)
        UpdateInfos(handles);

        %Update graphs in the GUI
        cla(handles.Plot1,'reset');
        cla(handles.Plot2,'reset');
        handles.CurrentRecording.drawTheta(handles.Plot1);
        handles.CurrentRecording.drawHandTraj(handles.Plot2);

        %Create other graphs
        handles.CurrentRecording.drawGlobalHandMaps(0);
        handles.CurrentRecording.drawStaticHandMaps(0);
        handles.CurrentRecording.drawMovHandMaps(0);
        handles.CurrentRecording.drawJointHists();
        
        guidata(hObject, handles);

        %And activate/deactivate stuff
        set(handles.Open,'enable','on');
        set(handles.Process,'enable','on');
        set(handles.PlayPause,'enable','off');
        set(handles.Load,'enable','on');
        set(handles.Save,'enable','off');
        set(handles.FilenameEdit,'string',filename);
        
        %Not busy anymore
        set(handles.figure1, 'pointer', 'arrow');
    end

    


% --- Executes on button press in Process.
function Process_Callback(hObject, eventdata, handles)
% hObject    handle to Process (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    
    %Ask user for start and end time in seconds and arm to process
    prompt = {'Effective sequence start (in s):','Effective sequence end (in s):','Arm side (L or R):'};
    dlg_title = 'Recording details';
    defaultans = {'60','600', 'R'};
    answer = inputdlg(prompt,dlg_title,1,defaultans);
    
    startt=str2double(answer{1});
    endt=str2double(answer{2});
    arm=answer{3};
    
    %Say we are busy
    set(handles.figure1, 'pointer', 'watch');
    drawnow;

    
    handles.CurrentRecording = Recording([handles.path handles.filename], startt, endt, 10, arm);
    
    %Update values in the GUI (nb movements, time)
    UpdateInfos(handles);

    %Calculate everything for the recording
    handles.CurrentRecording = handles.CurrentRecording.calcEverything();
    
    %Update values in the GUI (nb movements, time)
    UpdateInfos(handles);
    
    %Update graphs in the GUI
    cla(handles.Plot1,'reset');
    cla(handles.Plot2,'reset');
    handles.CurrentRecording.drawTheta(handles.Plot1);
    handles.CurrentRecording.drawHandTraj(handles.Plot2);
    
    %Create other graphs
    handles.CurrentRecording.drawGlobalHandMaps(0);
    handles.CurrentRecording.drawStaticHandMaps(0);
    handles.CurrentRecording.drawMovHandMaps(0);
    handles.CurrentRecording.drawJointHists();
    
    guidata(hObject, handles);
    
    %And activate/deactivate stuff
    set(handles.Open,'enable','on');
    set(handles.Process,'enable','on');
    set(handles.PlayPause,'enable','on');
    set(handles.Load,'enable','on');
    set(handles.Save,'enable','on');
    
    %Not busy anymore
    set(handles.figure1, 'pointer', 'arrow');

    
% --- Executes on button press in PlayPause.
function PlayPause_Callback(hObject, eventdata, handles)
% hObject    handle to PlayPause (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Open.
function Open_Callback(hObject, eventdata, handles)
% hObject    handle to Open (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    %Get file to open
    [filename, path] = uigetfile({'*.csv;*.h5'},'Data file');
    if(filename~=0)  
        handles.filename=filename;
        handles.path=path;
        guidata(hObject, handles);

        %And activate/deactivate stuff
        set(handles.Open,'enable','on');
        set(handles.Process,'enable','on');
        set(handles.PlayPause,'enable','off');
        set(handles.Load,'enable','on');
        set(handles.Save,'enable','off');
        set(handles.FilenameEdit,'string',filename);
    end

    
function UpdateInfos(handles)
    
    set(handles.NbMovTxt, 'string', ['Nb movements: ' num2str(handles.CurrentRecording.NbMov)]);
    set(handles.DurationTxt, 'string', ['Duration: ' num2str(handles.CurrentRecording.DurationMin) 'min ' num2str(handles.CurrentRecording.DurationSec) 's']);
    switch(handles.CurrentRecording.Arm)
        case 'L'
            set(handles.ArmTxt, 'string', 'Left arm');
        case 'R'
            set(handles.ArmTxt, 'string', 'Right arm');
    end


% --- Executes on button press in Save.
function Save_Callback(hObject, eventdata, handles)
% hObject    handle to Save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    %Replace file extension with a '.mat. and add ARM
    [pathstr,name,ext] = fileparts([handles.filename]);
    mat_filename=[pathstr name '_' handles.CurrentRecording.Arm '.mat'];
    
    
    %Save the current workspace
    save(mat_filename, '-struct', 'handles', 'CurrentRecording', 'filename', 'path');
    
    %Say it
    msgbox(['Current recording saved under ' mat_filename ' !'], 'Save'); 


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
    delete(hObject);
    close all;
