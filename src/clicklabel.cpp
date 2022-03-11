#include "../include/hg_gui/clicklabel.hpp"
#include <QMouseEvent>

ClickLabel::ClickLabel(QWidget *parent):
    QLabel(parent) ,m_pParent(parent)
{
    QPalette pa;
    pa.setColor(QPalette::WindowText,Qt::darkGreen);
    setPalette(pa);
}

ClickLabel::ClickLabel(const QString &text,QWidget *parent):
    QLabel(parent)
{
    setText(text);
}

void ClickLabel::mouseReleaseEvent(QMouseEvent *ev)
{
    if(ev->button() == Qt::LeftButton)
        emit clicked(user_graphic_cmd_index);
    else if(ev->button() == Qt::RightButton)
        emit del();
}

void ClickLabel::mousePressEvent(QMouseEvent *ev)
{
    if(ev->button() == Qt::LeftButton)
    {
        dragPosition = ev->globalPos()-frameGeometry().topLeft();
        ev->accept();
    }
}

void ClickLabel::mouseMoveEvent(QMouseEvent *ev)
{
    if(ev->buttons() & Qt::LeftButton)
    {
        m_MovePointX = (ev->globalPos()-dragPosition).x();
        m_MovePointY = (ev->globalPos()-dragPosition).y();
//        if(m_MovePointX < 0 || m_MovePointY < 0 || m_MovePointY > m_pParent->rect().bottom() - this->rect().width() || m_MovePointX > m_pParent->rect().right() - this->rect().width())
        if(m_MovePointX < 0 || m_MovePointY < 0 )
        {
            //防止移出顶部和底部
            if(m_MovePointY > m_pParent->rect().bottom() - this->rect().width() || m_MovePointY < 0)
            {
                move((ev->globalPos() - dragPosition).x(), this->geometry().y());
            }
            //防止移出左侧和右侧
            if(m_MovePointX > m_pParent->rect().right() - this->rect().width() || m_MovePointX < 0)
            {
                move(this->geometry().x(),(ev->globalPos() - dragPosition).y());
            }
            //防止移出左上角
            if(m_MovePointX < 0 && m_MovePointY < 0)
            {
                move(0,0);
            }
            //防止移出右小角
            if(m_MovePointY > m_pParent->rect().bottom() - this->rect().width() && m_MovePointX > m_pParent->rect().right() - this->rect().width())
            {
                move(m_pParent->rect().right() - this->rect().width(), m_pParent->rect().bottom() - this->rect().height());
            }
            //防止移出右上角
            if(m_MovePointY < 0 && m_MovePointX > m_pParent->rect().right() - this->rect().width())
            {
                move(m_pParent->rect().right() - this->rect().width(),0);
            }
            //防止移出左上角
            if(m_MovePointX < 0 && m_MovePointY > m_pParent->rect().bottom() - this->rect().height())
            {
                move(0,m_pParent->rect().bottom() - this->rect().height());
            }

        }
        else
        {
            move(ev->globalPos() - dragPosition);
        }
    }
}

void ClickLabel::enterEvent(QEvent *ev)
{
    QPalette pa;
    pa.setColor(QPalette::WindowText,Qt::blue);
    setPalette(pa);
}

void ClickLabel::leaveEvent(QEvent *ev)
{
    QPalette pa;
    pa.setColor(QPalette::WindowText,Qt::darkGreen);
    setPalette(pa);
}

