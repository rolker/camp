#ifndef CAMP_CACHED_FILE_LOADER_H
#define CAMP_CACHED_FILE_LOADER_H

#include <QObject>
#include <QDir>

class QNetworkAccessManager;
class QNetworkReply;

namespace camp
{

class MainWindow;

class CachedFileClient: public QObject
{
  Q_OBJECT
public:
  CachedFileClient(QObject* parent=nullptr);
signals:
  void dataLoaded(QByteArray &data, CachedFileClient* client);
};

// Loads file from a drive or
// from the network via http.
// can cache http files locally for performance.
class CachedFileLoader: public QObject
{
  Q_OBJECT
public:
  static CachedFileLoader* get();

  QDir cachePath() const;

public slots:
  void setCachePath(QString cache_path);
  void load(QString url, QString cache_local_path, CachedFileClient* client);

private:
  friend class MainWindow;
  CachedFileLoader(QObject* parent=nullptr);
  static void construct();
  static void destruct();

  static CachedFileLoader* instance;

  QNetworkAccessManager* network_access_manager_;

  // Base location where files are stored locally
  QString cache_path_;

private slots:
  void downloadFinished(QNetworkReply* reply);



};

} // namespace camp

#endif
