function varargout = Projekat3(varargin)
% PROJEKAT3 MATLAB code for Projekat3.fig
%      PROJEKAT3, by itself, creates a new PROJEKAT3 or raises the existing
%      singleton*.
%
%      H = PROJEKAT3 returns the handle to a new PROJEKAT3 or the handle to
%      the existing singleton*.
%
%      PROJEKAT3('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PROJEKAT3.M with the given input arguments.
%
%      PROJEKAT3('Property','Value',...) creates a new PROJEKAT3 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Projekat3_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Projekat3_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Projekat3

% Last Modified by GUIDE v2.5 15-May-2023 21:28:36

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Projekat3_OpeningFcn, ...
                   'gui_OutputFcn',  @Projekat3_OutputFcn, ...
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


% --- Executes just before Projekat3 is made visible.
function Projekat3_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Projekat3 (see VARARGIN)

% Choose default command line output for Projekat3
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Projekat3 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Projekat3_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton_load.
function pushbutton_load_Callback(hObject, eventdata, handles)
global eeg
global Fs
global t
global C4
global C3
global klase

EEG = load('EEG_zad3.mat')
eeg = EEG.eeg

%u literaturi je navedeno da se naši podaci nalaze u kolonama 5, 6 i 26

C3 = eeg(:,5);
C4 = eeg(:,6);
klase = eeg(:,26);

Fs = 160; %uslov zadatka
t = 0:1/Fs:(length(C3)-1)/Fs;%definisemo vreme


axes(handles.axes1)
plot(t,C3)
title('EEG signal,kanal C3')
xlim([0 746])
ylim([-500 500])
xlabel('vreme[s]')

axes(handles.axes2)
plot(t,C4)
title('EEG signal,kanal C4')
ylim([-500 500])
xlim([0 746])
xlabel('vreme[s]')


function pushbutton_filter_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_filter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Fs
global t
global C4
global C3

global C3_filtrirano
global C4_filtrirano


%filtriranje, ja koristim bandpass jer mi je smislenije nego lowpass koji
%je korišćen u naučnom radu onih ljudi 

[b,a] = butter(2,[8*2/Fs,30*2/Fs],'bandpass');
C3_filtrirano = filter(b,a,C3);
C4_filtrirano = filter(b,a,C4);


axes(handles.axes1)
plot(t,C3_filtrirano)
title('EEG signal - filtriran C3')
xlim([0 746])
ylim([-500 500])
xlabel('vreme[s]')

axes(handles.axes2)
plot(t,C4_filtrirano)
title('EEG signal - filtriran C4')
ylim([-500 500])
xlim([0 746])
xlabel('vreme[s]')


% --- Executes on button press in pushbutton_segm.
function pushbutton_segm_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_segm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA
%Generisanje skupa sa vektorima obeležja i njima odgovarajućim klasama
global Fs
global klase
global C3_filtrirano
global C4_filtrirano
global vektor_obelezja
global klase_sjedinjeno

%kreiranje praznih vektora
poceci_pokreta = [];
krajevi_pokreta = [];


for i = 1:length(klase)-1  

    %ukoliko prvi nije nula, a njegov sledbenik je nula, znači da je u
    %pitanju predviđanje pokreta 


    if klase(i) == 0 && klase(i+1) ~= 0 
        poceci_pokreta = [poceci_pokreta i+1];
    elseif (klase(i)==1||klase(i)==2) && klase(i+1)==0
        krajevi_pokreta = [krajevi_pokreta i];
    end

end

n = length(klase);
krajevi_pokreta = [krajevi_pokreta n];

alfa3 = [];
beta3 = [];
alfa4 = [];
beta4 = [];
klase_sjedinjeno = [];
 
for i = 1:length(poceci_pokreta)
    pocetak_pokreta = poceci_pokreta(i);
    kraj_pokreta = krajevi_pokreta(i);
        
    prozor_C3 = C3_filtrirano(pocetak_pokreta:kraj_pokreta); % prozor C3
    prozor_C4 = C4_filtrirano(pocetak_pokreta:kraj_pokreta); % prozor C4
     


%Kreiramo periodogram za C3 prozor
    [pxx3, F] = periodogram(prozor_C3, [], [], Fs);

    %definišemo frekvencije talasa

    alfa = F >= 8 & F <= 13;
    beta = F >= 13 & F <= 30;

    alfa3 = [alfa3 mean(pxx3(alfa))];
    beta3 = [beta3 mean(pxx3(beta))];


 %Kreiramo periodogram za C4 prozor

    [pxx4,F] = periodogram(prozor_C4, [], [], Fs);

    %definišemo frekvencije talasa
    alfa = F >= 8 & F <= 13;
    beta = F >= 13 & F <= 30;

    alfa4 = [alfa4 mean(pxx4(alfa))];
    beta4 = [beta4 mean(pxx4(beta))];
            
    prozor_klase = klase(pocetak_pokreta:kraj_pokreta);
    klase_sjedinjeno = [klase_sjedinjeno unique(prozor_klase)];  
end

vektor_obelezja = [alfa3; beta3; alfa4; beta4]; % 4x90
guidata(hObject, handles);


% --- Executes on button press in pushbutton_class.
function pushbutton_class_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_class (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vektor_obelezja
global klase_sjedinjeno



dugme1=get(handles.radiobutton1, 'Value');
dugme2= get(handles.radiobutton2, 'Value');
dugme3 = get(handles.radiobutton3, 'Value');

if  dugme1== 1
    m = 'linear';

elseif dugme2 == 1
    m = 'diaglinear';

elseif dugme3 == 1
    m = 'quadratic';
end

% Broj skupovau Cross Validaciji
k = 5;

% Broj uzoraka
uzorci = size(vektor_obelezja, 2);

% Broj uzoraka po skupu
samplesPerFold = floor(uzorci / k);

% Inicijalizacija niza za čuvanje tačnosti
tacnost = zeros(1, k);

% Inicijalizacija matrice konfuzije, kako bi mogao akumulirati
cm = zeros(numel(unique(klase_sjedinjeno)));

% Cross Validacija
for fold = 1:k
    % Odabir podataka za obuku i validaciju
    testIndices = (fold-1)*samplesPerFold + 1 : fold*samplesPerFold;
    trainIndices = setdiff(1:uzorci, testIndices);
    
    % Podela podataka za obuku i validaciju
    train_obelezja = vektor_obelezja(:, trainIndices)';
    train_klase = klase_sjedinjeno(trainIndices)';
    test_skup = vektor_obelezja(:, testIndices)';
    test_klase = klase_sjedinjeno(testIndices)';
    
    % Klasifikacija sa odabranim klasifikatorom
    klase_koje_smo_dobili = classify(test_skup, train_obelezja, train_klase,m);
    
    % Računanje tačnosti za svaki fols
    tacnost(fold) = sum(klase_koje_smo_dobili == test_klase) / numel(test_klase);
    % Ažuriranje matrice konfuzije
    cm = cm + confusionmat(test_klase, klase_koje_smo_dobili);

end

% Izračunavanje prosečne tačnosti
sr_tacnost = mean(tacnost);
set(handles.edit1, 'String', sprintf(' %.2f%%', sr_tacnost*100));

uitable1.RowName = {'Predviđene vrednosti klasa'};
set(handles.uitable1, 'Data', cm)


handles.output = hObject;
guidata(hObject, handles);



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
