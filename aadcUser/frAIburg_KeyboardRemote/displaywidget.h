/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must
display the following acknowledgement: “This product includes software developed
by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-12 09:39:39#$ $Rev:: 63110   $
**********************************************************************/

#ifndef DISPWIDGET_VALUEGENERATOR_FILTER
#define DISPWIDGET_VALUEGENERATOR_FILTER

#include "stdafx.h"
#include <QtCore/QtCore>
#include <QtGui/QtGui>
#include <boost/assign/list_of.hpp>
#ifdef WIN32
#ifdef _DEBUG
#pragma comment(lib, "qtmaind.lib")
#pragma comment(lib, "qtcored4.lib")
#pragma comment(lib, "qtguid4.lib")
#else  // _DEBUG
#pragma comment(lib, "qtmain.lib")
#pragma comment(lib, "qtcore4.lib")
#pragma comment(lib, "qtgui4.lib")
#endif
#endif

/*!
This class it the QWidget for the prototyp of the driver filter
*/
class DisplayWidget : public QWidget {
    Q_OBJECT

 public:
    /*! constructor for the widget
    * \param parent the parent widget
    */
    DisplayWidget(QWidget *parent);

    /*! Destructor. */
    ~DisplayWidget(){};

    /*! the button to send the response to the ready request in the given the
     * section */
    QPushButton *m_btSendValueTrue;

    /*! the button to send the error state */
    QPushButton *m_btSendValueFalse;

 signals:

    void keyReceived(int value);
    void sendoutFocusWidget();
    void sendpressUp();
    void sendpressDown();
    void sendpressLeft();
    void sendpressRight();
    void sendreleaseUp();
    void sendreleaseDown();
    void sendreleaseLeft();
    void sendreleaseRight();

 private:
    /*! the main widget */
    QWidget *m_pWidget;

    /*! the main layout for the widget*/
    QVBoxLayout *m_mainLayout;

 protected:
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void focusOutEvent(QFocusEvent *e);
};

#endif
