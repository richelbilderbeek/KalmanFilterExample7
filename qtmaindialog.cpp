#ifdef _WIN32
#undef __STRICT_ANSI__
#endif

#include "qtmaindialog.h"

#include <cassert>

#include <boost/lexical_cast.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>

#include "qwt_plot_curve.h"
#include "qwt_plot_grid.h"
#include "qwt_plot_zoomer.h"

#include "maindialog.h"
#include "matrix.h"
#include "ui_qtmaindialog.h"

QtMainDialog::QtMainDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::QtMainDialog),
  m_curve_v_estimate(new QwtPlotCurve),
  m_curve_v_measured(new QwtPlotCurve),
  m_curve_v_real(new QwtPlotCurve),
  m_curve_x_estimate(new QwtPlotCurve),
  m_curve_x_measured(new QwtPlotCurve),
  m_curve_x_real(new QwtPlotCurve)

{
  ui->setupUi(this);

  ui->plot_x->setTitle("Position");
  ui->plot_x->setAxisTitle(QwtPlot::xBottom,"Time");
  ui->plot_x->setAxisTitle(QwtPlot::yLeft,"Position");
  m_curve_x_estimate->setTitle("Estimated");
  m_curve_x_estimate->attach(ui->plot_x);
  m_curve_x_estimate->setStyle(QwtPlotCurve::Lines);
  m_curve_x_estimate->setPen(QPen(QColor(255,0,0)));
  m_curve_x_measured->setTitle("Measured");
  m_curve_x_measured->attach(ui->plot_x);
  m_curve_x_measured->setStyle(QwtPlotCurve::Lines);
  m_curve_x_measured->setPen(QPen(QColor(0,255,0)));
  m_curve_x_real->setTitle("Real");
  m_curve_x_real->attach(ui->plot_x);
  m_curve_x_real->setStyle(QwtPlotCurve::Lines);
  m_curve_x_real->setPen(QPen(QColor(0,0,255)));

  ui->plot_v->setTitle("Velocity");
  ui->plot_v->setAxisTitle(QwtPlot::xBottom,"Time");
  ui->plot_v->setAxisTitle(QwtPlot::yLeft,"Velocity");
  m_curve_v_estimate->setTitle("Estimated");
  m_curve_v_estimate->attach(ui->plot_v);
  m_curve_v_estimate->setStyle(QwtPlotCurve::Lines);
  m_curve_v_estimate->setPen(QPen(QColor(255,0,0)));
  m_curve_v_measured->setTitle("Measured");
  m_curve_v_measured->attach(ui->plot_v);
  m_curve_v_measured->setStyle(QwtPlotCurve::Lines);
  m_curve_v_measured->setPen(QPen(QColor(0,255,0)));
  m_curve_v_real->setTitle("Real");
  m_curve_v_real->attach(ui->plot_v);
  m_curve_v_real->setStyle(QwtPlotCurve::Lines);
  m_curve_v_real->setPen(QPen(QColor(0,0,255)));

  //Add grids
  { QwtPlotGrid * const grid = new QwtPlotGrid; grid->setPen(QPen(QColor(196,196,196))); grid->attach(ui->plot_x); }
  { QwtPlotGrid * const grid = new QwtPlotGrid; grid->setPen(QPen(QColor(196,196,196))); grid->attach(ui->plot_v); }
  //Add zoomers
  {
    new QwtPlotZoomer(ui->plot_x->canvas());
    new QwtPlotZoomer(ui->plot_v->canvas());
  }

  QObject::connect(ui->table_init_state_estimate,SIGNAL(cellChanged(int,int)),this,SLOT(OnAnyChange()));
  QObject::connect(ui->table_init_state_real,SIGNAL(cellChanged(int,int)),this,SLOT(OnAnyChange()));
  QObject::connect(ui->table_init_covariance_estimate,SIGNAL(cellChanged(int,int)),this,SLOT(OnAnyChange()));
  QObject::connect(ui->table_real_measurement_noise,SIGNAL(cellChanged(int,int)),this,SLOT(OnAnyChange()));
  QObject::connect(ui->table_control,SIGNAL(cellChanged(int,int)),this,SLOT(OnAnyChange()));
  QObject::connect(ui->table_real_process_noise,SIGNAL(cellChanged(int,int)),this,SLOT(OnAnyChange()));
  QObject::connect(ui->table_measurement_noise_estimate,SIGNAL(cellChanged(int,int)),this,SLOT(OnAnyChange()));
  QObject::connect(ui->table_observation,SIGNAL(cellChanged(int,int)),this,SLOT(OnAnyChange()));
  QObject::connect(ui->table_process_noise_covariance_estimate,SIGNAL(cellChanged(int,int)),this,SLOT(OnAnyChange()));
  QObject::connect(ui->table_state_transition,SIGNAL(cellChanged(int,int)),this,SLOT(OnAnyChange()));


  OnAnyChange();
}

QtMainDialog::~QtMainDialog()
{
  delete m_curve_v_estimate;
  delete m_curve_v_measured;
  delete m_curve_v_real;
  delete m_curve_x_estimate;
  delete m_curve_x_measured;
  delete m_curve_x_real;
  delete ui;
}

void QtMainDialog::OnAnyChange()
{

  try
  {
    //Do the sim
    const double acceleration = 1.0;

    const boost::numeric::ublas::vector<double> init_x_real = ToVector(ui->table_init_state_real);
    const boost::numeric::ublas::vector<double> x_real_measurement_noise = ToVector(ui->table_real_measurement_noise);
    const boost::numeric::ublas::vector<double> x_first_guess = ToVector(ui->table_init_state_estimate);
    const boost::numeric::ublas::matrix<double> p_first_guess = ToMatrix(ui->table_init_covariance_estimate);
    const boost::numeric::ublas::matrix<double> control = ToMatrix(ui->table_control);
    const boost::numeric::ublas::matrix<double> measurement_noise_estimate = ToMatrix(ui->table_measurement_noise_estimate);
    const boost::numeric::ublas::matrix<double> observation = ToMatrix(ui->table_observation);
    const boost::numeric::ublas::vector<double> real_process_noise = ToVector(ui->table_real_process_noise);
    const boost::numeric::ublas::matrix<double> process_noise_estimate = ToMatrix(ui->table_process_noise_covariance_estimate);
    const boost::numeric::ublas::matrix<double> state_transition = ToMatrix(ui->table_state_transition);

    const int time = 250;
    const MainDialog d(
      time,
      acceleration,
      control,
      measurement_noise_estimate,
      observation,
      p_first_guess,
      process_noise_estimate,
      state_transition,
      init_x_real,
      real_process_noise,
      x_first_guess,
      x_real_measurement_noise);

    //Display data
    {
      const boost::numeric::ublas::matrix<double>& data = d.GetData();
      std::vector<double> time_series(time);
      std::vector<double> v_estimate(time);
      std::vector<double> v_measured(time);
      std::vector<double> v_real(time);
      std::vector<double> x_estimate(time);
      std::vector<double> x_measured(time);
      std::vector<double> x_real(time);

      for (int row=0; row!=time; ++row)
      {
        time_series[row] = static_cast<double>(row);
        x_real[     row] = data(row,0);
        x_measured[ row] = data(row,1);
        x_estimate[ row] = data(row,2);
        v_real[     row] = data(row,3);
        v_measured[ row] = data(row,4);
        v_estimate[ row] = data(row,5);
      }
      m_curve_v_estimate->setData(new QwtPointArrayData(&time_series[0],&v_estimate[0],time_series.size()));
      m_curve_v_measured->setData(new QwtPointArrayData(&time_series[0],&v_measured[0],time_series.size()));
      m_curve_v_real->setData(new QwtPointArrayData(&time_series[0],&v_real[0],time_series.size()));
      m_curve_x_estimate->setData(new QwtPointArrayData(&time_series[0],&x_estimate[0],time_series.size()));
      m_curve_x_measured->setData(new QwtPointArrayData(&time_series[0],&x_measured[0],time_series.size()));
      m_curve_x_real->setData(new QwtPointArrayData(&time_series[0],&x_real[0],time_series.size()));
    }
    ui->plot_v->replot();
    ui->plot_x->replot();
  }
  catch (boost::bad_lexical_cast&)
  {
    const std::vector<double> v(1,0.0);
    m_curve_v_estimate->setData(new QwtPointArrayData(&v[0],&v[0],v.size()));
    m_curve_v_measured->setData(new QwtPointArrayData(&v[0],&v[0],v.size()));
    m_curve_v_real->setData(new QwtPointArrayData(&v[0],&v[0],v.size()));
    m_curve_x_estimate->setData(new QwtPointArrayData(&v[0],&v[0],v.size()));
    m_curve_x_measured->setData(new QwtPointArrayData(&v[0],&v[0],v.size()));
    m_curve_x_real->setData(new QwtPointArrayData(&v[0],&v[0],v.size()));
    ui->plot_v->replot();
    ui->plot_x->replot();

  }
}

const boost::numeric::ublas::matrix<double> QtMainDialog::ToMatrix(const QTableWidget * const table)
{
  assert(table);
  const int n_rows = table->rowCount();
  const int n_cols = table->columnCount();
  boost::numeric::ublas::matrix<double> v(n_rows,n_cols);
  for(int col=0;col!=n_cols;++col)
  {
    for(int row=0;row!=n_rows;++row)
    {
      const auto item = table->item(row,col);
      assert(item);
      const std::string text = item->text().toStdString();
      v(row,col) = boost::lexical_cast<double>(text);
    }
  }
  return v;
}

const boost::numeric::ublas::vector<double> QtMainDialog::ToVector(const QTableWidget * const table)
{
  assert(table);
  assert(table->columnCount() == 1);
  const int n_rows = table->rowCount();
  boost::numeric::ublas::vector<double> v(n_rows);
  for(int row=0;row!=n_rows;++row)
  {
    const auto item = table->item(row,0);
    assert(item);
    const std::string text = item->text().toStdString();
    v(row) = boost::lexical_cast<double>(text);
  }
  return v;

}
