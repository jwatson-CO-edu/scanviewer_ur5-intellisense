% hw_grad.cls
% variation of jstrom-short.cls from Joho Strom
% inspired by resume.cls from dburrows
%

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{hw_grad}

\LoadClassWithOptions{article}

  \RequirePackage{calc}            % Needed for settoheight manipulations

\usepackage{graphicx}    % needed for including graphics e.g. EPS, PS
\topmargin -1.5cm        % read Lamport p.163
\oddsidemargin -0.04cm   % read Lamport p.163
\evensidemargin -0.04cm  % same as oddsidemargin but for left-hand pages
\textwidth 16.59cm
\textheight 21.94cm
\parskip 7.2pt           % sets spacing between paragraphs
\renewcommand{\baselinestretch}{1.0} % 1.0 (was 1.5) spacing between lines
\parindent 0pt		  % sets leading space for paragraphs
\pagenumbering{arabic}
\usepackage[all, cmtip]{xy} % Graphs! 
\usepackage{listings}
\usepackage{ifthen}
\usepackage{ragged2e}
\RaggedRight

% ==== HW Repair ====
% Version 2016-12-09

% == Imports ==
\usepackage{epstopdf} % Process EPS from matplotlib --> PDF
\usepackage{verbatim} % Block Comment: \begin{comment} ... \end{comment}
% \usepackage{minted} % code listings % URL, Settings to use this package: http://tex.stackexchange.com/a/99476
\usepackage[margin=0.5in]{geometry} % Override enormous margins! 
\usepackage{framed} % frame a lot of stuff, URL: http://tex.stackexchange.com/a/163376
\usepackage{tkz-graph} % For drawing graphs
\usepackage{tikz}
\usetikzlibrary{arrows}
\usetikzlibrary{calc}
\usepackage{amsmath}
\usepackage{wasysym} %Load the package 'wasysym' after the 'ams'packages , URL: http://tex.stackexchange.com/a/183364/115244 % $ \RIGHTarrow $
% == End Imports ==

% == New Commands ==
\newcommand{\crcchr}[1]{{\Large \textcircled{\normalsize #1}}} % Circle a characher for diagrams/graphs
\newcommand{\crcmth}[1]{{\LARGE \textcircled{\normalsize #1}}} % Circle large enough for a negated character
\newcommand{\dspst}{\displaystyle} % Shortcut for \displaystyle
\newcommand{\BU}[1]{\underline{\textbf{#1}}} % \BU - bold and underline
\newcommand{\lfb}{\left[}  
\newcommand{\rtb}{\right]}
\newcommand{\lfp}{\left(}  
\newcommand{\rtp}{\right)}
\newcommand{\rarw}{$\rightarrow$\ }
\newcommand{\larw}{$\leftarrow$\ }
\newcommand{\mbf}[1]{\mathbf{#1}}
\newcommand{\ans}{\textbf{Answer}:\ }
% == End Commands ==

% == Commands Definitions ==
% ~ Bold Vectors & Matrices ~
\def\vphi{\hbox{\boldmath$\phi$}}
\def\veta{\hbox{\boldmath$\eta$}}
\def\vpsi{\hbox{\boldmath$\psi$}}
\def\vA{\hbox{\boldmath$A$}}
\def\vb{\hbox{\boldmath$b$}}
\def\vc{\hbox{\boldmath$c$}}
\def\vC{\hbox{\boldmath$C$}}
\def\vd{\hbox{\boldmath$d$}}
\def\vepsilon{\hbox{\boldmath$\epsilon$}}
\def\vf{\hbox{\boldmath$f$}}
\def\vg{\hbox{\boldmath$g$}}
\def\vI{\hbox{\boldmath$I$}}
\def\vJ{\hbox{\boldmath$J$}}
\def\vK{\hbox{\boldmath$K$}}
\def\vl{\hbox{\boldmath$l$}}
\def\vL{\hbox{\boldmath$L$}}
\def\vn{\hbox{\boldmath$n$}}
\def\vp{\hbox{\boldmath$p$}}
\def\vq{\hbox{\boldmath$q$}}
\def\vr{\hbox{\boldmath$r$}}
\def\vR{\hbox{\boldmath$R$}}
\def\vS{\hbox{\boldmath$S$}}
\def\vt{\hbox{\boldmath$t$}}
\def\vtau{\hbox{\boldmath$\tau$}}
\def\vv{\hbox{\boldmath$v$}}
\def\vV{\hbox{\boldmath$V$}}
\def\vw{\hbox{\boldmath$w$}}
\def\vW{\hbox{\boldmath$W$}}
\def\vx{\hbox{\boldmath$x$}}
\def\vy{\hbox{\boldmath$y$}}
\def\vz{\hbox{\boldmath$z$}}
\def\vomega{\hbox{\boldmath$\omega$}}
% ~ Special Matrices ~
\def\eye{\hbox{\boldmath$I$}}
\def\ones{\hbox{\boldmath$1$}}
\def\Rot{\hbox{\boldmath$R$}}
\def\Trans{\hbox{\boldmath$T$}}
\def\vzero{\hbox{\boldmath$0$}}
% ~ Tabular Formatting ~
\def\topCellPad{\rule{0pt}{2.6ex}}
% ~ Left Scripts ~
\newcommand\lsup[1]{\!^{#1}\!}
% ~ Special Functions ~
\DeclareMathOperator{\atan}{atan}
\DeclareMathOperator{\atan2}{atan2}
\DeclareMathOperator{\acos}{acos}
\DeclareMathOperator{\sign}{sign}
\DeclareMathOperator{\dor}{dor}
\DeclareMathOperator{\median}{median}
% ~ Special Notations ~
\newcommand{\norm}[1]{\left\lVert#1\right\rVert} % URL , Norm: https://tex.stackexchange.com/a/107190
% __ End Definitions __

% == Python Environment == 
% \begin{lstlisting}[language=python] 
% \end{lstlisting}
\usepackage{listings} % source code formatting, URL: https://www.overleaf.com/4950285xjfphj#/15273478/
\usepackage{color}
\usepackage{graphicx} % Include a picture: \includegraphics[scale=0.45]{fileNameWithoutEXT}

\definecolor{mygreen}{rgb}{0,0.6,0} % colors to be used for highlighting
\definecolor{mygray}{rgb}{0.5,0.5,0.5}
\definecolor{mymauve}{rgb}{0.58,0,0.82}

\lstset{ %
	tabsize=2,
	backgroundcolor=\color{white},% ---- choose the background color
	basicstyle=\footnotesize\ttfamily, % size of fonts used for the code
	breaklines=true, % ----------------- automatic line breaking only at whitespace
	captionpos=b, % -------------------- sets the caption-position to bottom
	commentstyle=\color{mygreen}, % ---- comment style
	escapeinside={\%*}{*)}, % ---------- if you want to add LaTeX within your code
	keywordstyle=\color{blue}, % ------- keyword style
	stringstyle=\color{mymauve}, % ----- string literal style
}
% == End Python ==
% ==== End Repair ====

  %%%%%%%%%%%%%%%%% define variables for the title
  \newcommand{\@authorname}{}
  \renewcommand{\@title}{}
  \newcommand{\@class}{}
  \newcommand{\@duedate}{}
  \newcommand{\@institute}{}

  \renewcommand{\author}[1]{\renewcommand{\@authorname}{#1}}
  \renewcommand{\title}[1]{\renewcommand{\@title}{#1}}
  \newcommand{\class}[1]{\renewcommand{\@class}{#1}}
  \newcommand{\duedate}[1]{\renewcommand{\@duedate}{#1}}
  \newcommand{\institute}[1]{\renewcommand{\@institute}{#1}}



% TODO: Make writing of second line conditional on \duedate and \class being defined
% TODO: Same for institute
  %%%%% Define a table to hold the author, etc to be placed in title
  \newcommand{\authortable}{
    \begin{tabular}{@{} r @{}}
      \@authorname \\
      \@duedate\\
    \end{tabular}
  }

  %%%%% Over ride the \maketitle command %%%%%
  \renewcommand{\maketitle}{
    \par
    %%%%% Discover the size of the author box so we can
    %%%%% give the correct amount of space to the title
    \newlength{\authorwidth}
    \settowidth{\authorwidth}{\authortable}
    %%%%% Place the title and the author box side-by-side
    \noindent
    \parbox{
      (\textwidth)}{\bf{\Large \@title} \\
      \@class \hfill  \@authorname\\
      \@institute \hfill \@duedate
    }  \newline
    % \mbox{ \authortable }
    %%%%% Find height of the current font
    \newlength{\fontheight}
    \settoheight{\fontheight}{A}
    %%%%% Draw a horizontal line
    \noindent \rule[\fontheight]{\textwidth}{.5pt}
  }
