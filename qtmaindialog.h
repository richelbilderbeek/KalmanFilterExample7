#ifndef QTMAINDIALOG_H
#define QTMAINDIALOG_H

#ifdef _WIN32
#undef __STRICT_ANSI__
#endif

#include <QDialog>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>

namespace Ui {
  class QtMainDialog;
}

struct QwtPlotCurve;
struct QTableWidget;

class QtMainDialog : public QDialog
{
  Q_OBJECT
  
public:
  explicit QtMainDialog(QWidget *parent = 0);
  ~QtMainDialog();
  
private:
  Ui::QtMainDialog *ui;

  QwtPlotCurve * const m_curve_v_estimate;
  QwtPlotCurve * const m_curve_v_measured;
  QwtPlotCurve * const m_curve_v_real;
  QwtPlotCurve * const m_curve_x_estimate;
  QwtPlotCurve * const m_curve_x_measured;
  QwtPlotCurve * const m_curve_x_real;

  static const boost::numeric::ublas::matrix<double> ToMatrix(const QTableWidget * const table);
  static const boost::numeric::ublas::vector<double> ToVector(const QTableWidget * const table);

private slots:
  void OnAnyChange();
};

#endif // QTMAINDIALOG_H
