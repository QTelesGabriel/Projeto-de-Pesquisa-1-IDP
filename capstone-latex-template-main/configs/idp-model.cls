\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{idp-model}[2025/04/28 Classe para trabalhos Tech IDP]
\LoadClass[12pt, a4paper]{article}
\RequirePackage[a4paper, left=3cm, right=2cm, top=3cm, bottom=2cm,
headheight=1.8cm]{geometry}

\RequirePackage{eso-pic}
\RequirePackage{graphicx}
\RequirePackage{wallpaper}
\RequirePackage{anyfontsize}
\RequirePackage{fontspec}
\setmainfont{Arial}
\RequirePackage{etoolbox}
\RequirePackage{titlesec}
\RequirePackage{tikz}
\RequirePackage{fancyhdr}
\RequirePackage{titlesec}
\RequirePackage{xcolor}
\RequirePackage{setspace}
\RequirePackage{titletoc}
\RequirePackage{hyperref}
\RequirePackage[most]{tcolorbox}

\RequirePackage{cite} % Pacote para formatação numérica padrão IEEE
\bibliographystyle{IEEEtran} % Estilo de bibliografia IEEE


\RequirePackage[english]{babel}

\onehalfspacing



\RequirePackage{lipsum}

\definecolor{fundo_idp}{HTML}{005DAA}
\definecolor{titulos}{HTML}{00B3FF}

\makeatletter

%% --- INÍCIO: macros seguras (sem @) ---
% Curso
\newcommand{\nomecurso}{} % valor por defeito vazio
\newcommand{\cienciadacomputacao}{%
  \renewcommand{\nomecurso}{Ciência da Computação}%
}
\newcommand{\engenhariadesoftware}{%
  \renewcommand{\nomecurso}{Engenharia de Software}%
}
% Se precisar colocar em inglês
\newcommand{\nomecursoingles}{} % valor por defeito vazio
\newcommand{\computerscience}{%
  \renewcommand{\nomecursoingles}{Computer Science}%
}
\newcommand{\softwareengineering}{%
  \renewcommand{\nomecursoingles}{Software Engineering}%
}



% Título, autor, ano (já existiam; mantemos)
\newcommand{\titulo}[1]{\def\titulonome{#1}}
\newcommand{\autor}[1]{\def\autornome{#1}}
\newcommand{\ano}[1]{\def\anonome{#1}}

% ORIENTADOR / ORIENTADORA
\newcommand{\nomeorientador}{}        % nome do orientador(a)
\newcommand{\instorientador}{IDP}    % instituição do orientador, padrão IDP
\newif\iforientadora                 % flag: true = orientadora

% Orientador (masculino) com argumento opcional [instituição]{nome}
\newcommand{\orientador}[2][]{%
  \orientadorafalse
  \renewcommand{\nomeorientador}{#2}%
  % se #1 vazio -> IDP, senão usa #1
  \renewcommand{\instorientador}{\ifx\relax#1\relax IDP\else #1\fi}%
}

% Orientadora (feminino) com argumento opcional [instituição]{nome}
\newcommand{\orientadora}[2][]{%
  \orientadoratrue
  \renewcommand{\nomeorientador}{#2}%
  \renewcommand{\instorientador}{\ifx\relax#1\relax IDP\else #1\fi}%
}

% Membros da banca (nomes, função, instituição) — versões com 3 args
\newcommand{\nomemembrobancai}{}   \newcommand{\funcaomembrobancai}{} \newcommand{\instbancai}{IDP}
\newcommand{\nomemembrobancaii}{}  \newcommand{\funcaomembrobancaii}{} \newcommand{\instbancaii}{IDP}

\newcommand{\membrobancai}[3]{%
  \renewcommand{\nomemembrobancai}{#1}%
  \renewcommand{\funcaomembrobancai}{#2}%
  \renewcommand{\instbancai}{#3}%
}

\newcommand{\membrobancaii}[3]{%
  \renewcommand{\nomemembrobancaii}{#1}%
  \renewcommand{\funcaomembrobancaii}{#2}%
  \renewcommand{\instbancaii}{#3}%
}

% Data de aprovação
\newcommand{\dataaprovacaoVal}{} % valor físico
\newcommand{\dataaprovacao}[1]{\renewcommand{\dataaprovacaoVal}{#1}}

% Palavras-chave / keywords
\newcommand{\palavraschaveVal}{} \newcommand{\keywordsVal}{}
\newcommand{\palavraschave}[1]{\renewcommand{\palavraschaveVal}{#1}}
\newcommand{\keywords}[1]{\renewcommand{\keywordsVal}{#1}}
%% --- FIM: macros seguras (sem @) ---


\makeatother

\fancyhf{} % Limpa os cabeçalhos e rodapés padrão
\renewcommand{\headrulewidth}{0pt} % Remove a linha do cabeçalho


\fancypagestyle{paginas}{
    \fancyhf{}
   \fancyhead[L]{%
    \hspace*{-3cm} % Compensa a margem esquerda do geometry
    \includegraphics[width=\paperwidth]{configs/figuras/cabecalho.png}
  }%

  
\fancyfoot[C]{%
  \begin{tikzpicture}
    \filldraw[fill=fundo_idp, draw=fundo_idp] (0,0) circle (0.4cm);
    \node[text=white] at (0,0) {\thepage};
  \end{tikzpicture}%
}
}

\fancypagestyle{pretext}{
    \fancyhf{}
   \fancyhead[L]{%
    \hspace*{-3cm} % Compensa a margem esquerda do geometry
    \includegraphics[width=\paperwidth]{configs/figuras/cabecalho.png}
  }%
}

\tcbset{
  keywordsbox/.style={
    colback=blue!5,
    colframe=blue!5, % sem borda visível
    boxrule=0pt,
    arc=2pt,
    left=4pt,
    right=4pt,
    top=2pt,
    bottom=2pt,
    boxsep=4pt,
    before skip=10pt,
    after skip=10pt,
  }
}

\pagestyle{paginas}




\newcommand{\capaprincipal}{
    \color{white}
    \begin{titlepage}
    

    \vspace*{8.5 cm}
    \begin{flushleft}
    {\Large\expandafter\MakeUppercase\expandafter{Bacharelado em}}\\
    \vspace*{0.2cm}
    {\huge\bf\expandafter\MakeUppercase\expandafter{\nomecurso}}

    \vspace*{2.0 cm}
        \AddToShipoutPicture*{
    \put(-3,0){
    \includegraphics[width=\paperwidth,height=\paperheight]{configs/figuras/capa_tcc.pdf}
    }}
{\Large\bf\expandafter\MakeUppercase\expandafter{\titulonome}}

    
    \end{flushleft}
    \vspace*{6 cm}
    \begin{center}
    {\Large\bf\expandafter\MakeUppercase\expandafter{\autornome}}
    \vfill
    {Brasília - DF, \anonome}
    
    \end{center}
    \end{titlepage}
    \color{black}


}

\newcommand{\folharosto}{
    \clearpage
    \thispagestyle{pretext}
    \begin{flushleft}
        {\Large\bfseries\color{fundo_idp}\MakeUppercase\autornome}

        \vspace*{3cm}
        {\Large\color{fundo_idp}\MakeUppercase\titulonome}

        \vspace*{3cm}
 
        \noindent{Undergraduate Thesis submitted in partial fulfillment of the
        requirements for the degree of Bachelor of \nomecursoingles{} at the
        Brazilian Institute of Education, Development and Research (IDP).}

        \vspace*{3cm}

        {\bfseries\color{fundo_idp}
        \iforientadora Advisor
        \else Advisor
        \fi}\\
        {\nomeorientador}

        \vspace*{3cm}
        {Brasília - DF, \anonome}
    \end{flushleft}
}

\newcommand{\folhaaprovacao}{
    \clearpage
    \thispagestyle{pretext}
   \begin{flushleft}
        {\Large\bfseries\color{fundo_idp}\MakeUppercase\autornome}

        \vspace*{3cm}
        {\Large\color{fundo_idp}\MakeUppercase\titulonome}

    \end{flushleft}
        \vspace*{2cm}

        \noindent{Undergraduate Thesis submitted in partial fulfillment of the
        requirements for the degree of Bachelor of \nomecursoingles{} at the
        Brazilian Institute of Education, Development and Research (IDP).}

    \begin{flushleft}

        \vspace*{2cm}
        {Approved on \dataaprovacaoVal}

        \vspace*{1cm}
        {\bfseries\color{fundo_idp}Examining Committee}
        
        
        {
        \vspace*{2cm}
        % \rule{\linewidth}{0.4pt}
        \vspace{1em}
        {\nomeorientador\ (\iforientadora Advisor\else Advisor\fi) - \instorientador}

        \vspace*{2cm}
        % \rule{\linewidth}{0.4pt}
        \vspace{1em}
        {\nomemembrobancai\ (\funcaomembrobancai) - \instbancai}
        
        \vspace*{2cm}
        % \rule{\linewidth}{0.4pt}
        \vspace{1em}
        {\nomemembrobancaii\ (\funcaomembrobancaii) - \instbancaii}
        }
    \end{flushleft}
}
\newcommand{\agradecimentos}{
    \clearpage
     \begin{flushleft}
    {\Large\bfseries\color{fundo_idp}\MakeUppercase{Agradecimentos}}
    \end{flushleft}
    \vspace{1em}
    \input{partes/agradecimentos}
}

\newcommand{\dedicatoria}{
    \clearpage
     \begin{flushleft}
    {\Large\bfseries\color{fundo_idp}\MakeUppercase{Dedicatória}}
    \end{flushleft}
    \vspace{1em}
    \input{partes/dedicatoria}
}

\newcommand{\resumo}{
    \clearpage
     \begin{flushleft}
    {\Large\bfseries\color{fundo_idp}\MakeUppercase{Resumo}}
    \end{flushleft}
    \vspace{1em}
    \input{partes/resumo}
    \vspace{1em}
     \begin{tcolorbox}[keywordsbox]
     \begin{center}
        \color{fundo_idp}\textbf{Palavras-chave:}\color{fundo_idp}{\palavraschaveVal}.
    \end{center}
    \end{tcolorbox}
}

\renewcommand{\abstract}{
    \clearpage
     \begin{flushleft}
    {\Large\bfseries\color{fundo_idp}\MakeUppercase{Abstract}}
    \end{flushleft}
    \vspace{1em}
    \input{partes/abstract}
    \vspace{1em}
     \begin{tcolorbox}[keywordsbox]
        \begin{center}
            \color{fundo_idp}\textbf{Keywords:} \color{fundo_idp}{\keywordsVal}.
        \end{center}
    \end{tcolorbox}
    \clearpage
}


\newcommand{\capafinal}{
\clearpage
\thispagestyle{empty}
 \vspace*{2.0 cm}
        \AddToShipoutPicture*{
    \put(-3,0){
    \includegraphics[width=\paperwidth,height=\paperheight]{configs/figuras/capa_final.pdf}
    }}

\clearpage
}


% Comando para a "página de abertura de seção" CORRIGIDO
\newcommand{\chapterbreak}[2]{%
    \clearpage
    \thispagestyle{empty}
    \AddToShipoutPicture*{
        \put(-3,0){
        \includegraphics[width=\paperwidth,height=\paperheight]{configs/figuras/capa_secoes.pdf}
    }}
    \color{white}
    \vspace*{\fill}
    \begin{center}
        {\fontsize{400}{480}\bfseries #1}
    \end{center}
    \vspace*{\fill}
    \clearpage
    \phantomsection
    \addcontentsline{toc}{section}{\protect\numberline{#1}#2} % CORREÇÃO PRINCIPAL
    \color{black}
}

\newcounter{secnum}

\newcommand{\secao}[2]{%
  \refstepcounter{secnum}%
  \chapterbreak{\arabic{secnum}}{#1}%
  
  \noindent
  \hspace*{-3.5cm}
  \begin{tikzpicture}
    \fill[cyan!70!blue!60] (0,0) rectangle (4,4);
    \node[white, font=\bfseries\fontsize{80}{80}\selectfont] at (2,2) {\arabic{secnum}};
  \end{tikzpicture}%
  \hspace{1em}%
  \begin{minipage}[t]{0.75\textwidth}
    \vspace{-0.8cm}
    {\color{titulos}\huge\bfseries\MakeUppercase{#1}}\par\vspace{1ex} % Adicionei {} aqui
  \end{minipage}
  \vspace{2em}


  \input{#2}
}

%%-------SUBSECOES--------------%%
\newcounter{subsecnum}[secnum]
\renewcommand{\thesubsecnum}{\arabic{secnum}.\arabic{subsecnum}}

\newcommand{\subsecao}[1]{%
  \refstepcounter{subsecnum}
  \par\vspace{1.5ex}

      \phantomsection
      \addcontentsline{toc}{subsection}{\arabic{secnum}.\arabic{subsecnum} #1} % <-- Adiciona ao sumário

  {\large\noindent\color{fundo_idp}\bfseries\MakeUppercase{\thesubsecnum\quad #1}}\par
  \vspace{1ex}
}


% Define o contador de subsubseções
\newcounter{subsubsecnum}[subsecnum]
\renewcommand{\thesubsubsecnum}{\arabic{secnum}.\arabic{subsecnum}.\arabic{subsubsecnum}}

% Define o comando para subsubseções
\newcommand{\subsubsecao}[1]{%
  \refstepcounter{subsubsecnum}
  \par\vspace{1.2ex}
  \phantomsection
  \addcontentsline{toc}{subsubsection}{\thesubsubsecnum #1} % <-- Adiciona ao sumário

  {\normalsize\noindent\color{fundo_idp}\bfseries\thesubsubsecnum\quad #1}\par
  \vspace{0.5ex}
}


% REFERENCIAS
\newcommand{\capareferencias}{
    \clearpage
    \thispagestyle{empty}
    \AddToShipoutPicture*{
        \put(-3,0){
            \includegraphics[width=\paperwidth,height=\paperheight]{configs/figuras/capa_secoes.pdf}
        }
    }
     \color{white}

    \vspace*{\fill}
    \begin{center}

    {\fontsize{60}{0}\bfseries\MakeUppercase{References}}
    \end{center}
    \vspace*{\fill}

    \refstepcounter{secnum}
    \phantomsection
    \addcontentsline{toc}{section}{References}

    \color{black}
    
    \null
    \clearpage
}

\newcommand{\referencias}{
    \capareferencias
    \bibliography{referencias} % Nome do arquivo .bib
}

\addto\captionsportuguese{
  \renewcommand{\refname}{%
    \color{titulos}\bfseries\MakeUppercase{References}
  }
}

% SUMÁRIO
% Altera o título do sumário
\addto\captionsportuguese{%
  \renewcommand{\contentsname}{\color{white}\fontsize{40}{0}\bfseries\MakeUppercase{Sumário}}
}
\addto\captionsenglish{%
  \renewcommand{\contentsname}{\color{white}\fontsize{40}{0}\bfseries\MakeUppercase{Contents}}
}


\addto\captionsportuguese{%
  \renewcommand{\listfigurename}{\color{white}\fontsize{40}{0}\bfseries\MakeUppercase{Lista de Figuras}}
  \renewcommand{\listtablename}{\color{white}\fontsize{40}{0}\bfseries\MakeUppercase{Lista de Tabelas}}
}

\addto\captionsenglish{%
  \renewcommand{\listfigurename}{\color{white}\fontsize{40}{0}\bfseries\MakeUppercase{List of Figures}}
  \renewcommand{\listtablename}{\color{white}\fontsize{40}{0}\bfseries\MakeUppercase{List of Tables}}
}




% Estilo dos itens da lista de figuras
\titlecontents{figure}[0pt]
{\vspace{0.5ex}\bfseries\large\color{white}}
{\contentslabel{2em}}
{}
{\titlerule*[0.5pc]{.}\contentspage}

% Estilo dos itens da lista de tabelas
\titlecontents{table}[0pt]
{\vspace{0.5ex}\bfseries\large\color{white}}
{\contentslabel{2em}}
{}
{\titlerule*[0.5pc]{.}\contentspage}

% Estilo do sumário
\titlecontents{section}[0pt]
{\vspace{0.5ex}\bfseries\large\color{white}}
{\contentslabel{2em}}
{}
{\titlerule*[0.5pc]{.}\contentspage}



\titlecontents{subsection}[2em]
{\vspace{0.3ex}\normalsize\color{white}}
{\contentslabel{2.3em}}
{}
{\titlerule*[0.5pc]{.}\contentspage}

\titlecontents{subsubsection}
  [4.5em] % Indentação ainda maior
  {\color{white}\addvspace{2pt}\small\itshape} % Formatação diferenciada
  {\contentslabel{3em}} % Número
  {} % Sem prefixo
  {\titlerule*[.5pc]{.}\contentspage} % Página



\newcommand{\backgroundsumario}{%
    \AddToShipoutPictureBG{%
        \put(0,0){%
            \includegraphics[width=\paperwidth,height=\paperheight]{configs/figuras/capa_secoes.pdf}%
        }%
    }%
}

\newenvironment{sumariolistas}{%
    \backgroundsumario
    \pagestyle{empty}%
}{%
    \ClearShipoutPictureBG
}

\makeatletter
% \renewcommand\tableofcontents{%
%     \cleardoublepage
%     \thispagestyle{empty} % Remove cabeçalhos e rodapés
%     \backgroundsumario % Ativa o fundo apenas nesta página
%     \section*{\contentsname} % Título do sumário
%     \@starttoc{toc} % Conteúdo do sumário
   
%     \ClearShipoutPictureBG % Remove o fundo das páginas seguintes
% }

\renewcommand\tableofcontents{%
    \cleardoublepage
   \begingroup
   \backgroundsumario
   \pagestyle{empty}
    \section*{\contentsname}
    \@starttoc{toc}
    \clearpage
    \endgroup
    \ClearShipoutPictureBG
}
\makeatother

\makeatletter
\renewcommand\listoffigures{%
    \cleardoublepage
   \begingroup
   \backgroundsumario
   \pagestyle{empty}
    \section*{\listfigurename}
    \@starttoc{lof}
     \clearpage
    \endgroup
    \ClearShipoutPictureBG
}

\renewcommand\listoftables{%
    \cleardoublepage
   \begingroup
   \backgroundsumario
   \pagestyle{empty}
    \section*{\listtablename}
    \@starttoc{lot}
     \clearpage
    \endgroup
    \ClearShipoutPictureBG
}
\makeatother

% APENDICE
\newcommand{\capaapendice}[1]{
    \clearpage
    \thispagestyle{empty}
    \AddToShipoutPicture*{
        \put(-3,0){
            \includegraphics[width=\paperwidth,height=\paperheight]{configs/figuras/capa_secoes.pdf}
        }
    }
     \color{white}

    \vspace*{\fill}
    \begin{center}

    {\fontsize{60}{0}\bfseries\MakeUppercase{#1}}
    \end{center}
    \vspace*{\fill}

    \refstepcounter{secnum}
    \phantomsection
    \addcontentsline{toc}{section}{#1}

    \color{black}
    
    \null
    \clearpage
}

\newcounter{apendicecount}
\setcounter{apendicecount}{0}%

\makeatletter
\newcommand{\labelapendice}[1]{%
        \edef\@currentlabel{\MakeUppercase{\alph{apendicecount}}}%
        \label{#1}%
}
\makeatother

\newcommand{\apendice}[3][]{%
        \clearpage
        \stepcounter{apendicecount}%
        \ifnum\value{apendicecount}=1
                \capaapendice{Appendices}
        \fi
        \phantomsection
        \addcontentsline{toc}{section}{Appendix \MakeUppercase{\alph{apendicecount}} - #2}
        \begin{center}
                {\color{fundo_idp}\Large{Appendix \MakeUppercase{\alph{apendicecount}} - #2}}
        \end{center}
        \ifx&#1&%
                \edef\tempaplabel{ap:\alph{apendicecount}}%
                \labelapendice{\tempaplabel}%
        \else
                \labelapendice{#1}%
        \fi
        \input{#3}
}

%% Anexos


\newcounter{anexocount}
\setcounter{anexocount}{0}%

\newcommand{\anexo}[2]{
        \clearpage
        \stepcounter{anexocount}%
        \ifnum\value{anexocount}=1
        \capaapendice{Annexes}
        \fi
        \phantomsection
        \addcontentsline{toc}{subsection}{\MakeUppercase{\alph{anexocount}}\hspace{1em} #1}
        \begin{center}
        {\color{fundo_idp}\Large{Annex \MakeUppercase{\alph{anexocount}}} - {#1}}
        \end{center}
        
        \input{#2}
}


