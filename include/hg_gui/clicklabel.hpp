#ifndef CLICKLABEL_H
#define CLICKLABEL_H

#include <QObject>
#include <QLabel>
#include <QDebug>

/*
* @clss Name     ClickLabel
* @description   具有单击相应的标签类
* @author        吴凯荣
* @date          2021-09-30
*/

extern int user_graphic_cmd_index;

class ClickLabel : public QLabel
{
    Q_OBJECT
public:
    explicit ClickLabel(QWidget *parent=0);
    ClickLabel(const QString &text,QWidget *parent=0);

signals:
    // 鼠标单击信号
    void clicked(int);
    void del();

protected:
    // 鼠标单击事件
    void mouseReleaseEvent(QMouseEvent *ev);
    void mousePressEvent(QMouseEvent *ev);
    void mouseMoveEvent(QMouseEvent *ev);
    void enterEvent(QEvent *ev);
    void leaveEvent(QEvent *ev);

public slots:

private:
    QPoint dragPosition;
    QWidget *m_pParent;
    int m_MovePointX;
    int m_MovePointY;

};

#endif // CLICKLABEL_H

