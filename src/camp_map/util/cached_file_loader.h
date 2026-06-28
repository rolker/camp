#ifndef CAMP_CACHED_FILE_LOADER_H
#define CAMP_CACHED_FILE_LOADER_H

#include <QObject>
#include <QDir>

class QApplication;
class QNetworkAccessManager;
class QNetworkReply;

namespace camp
{

class CachedFileClient: public QObject
{
  Q_OBJECT
public:
  CachedFileClient(QObject* parent=nullptr);
signals:
  void dataLoaded(QByteArray &data, CachedFileClient* client);
};

/// Loads file from a drive or
/// from the network via http.
/// Can cache http files locally for performance.
class CachedFileLoader: public QObject
{
  Q_OBJECT
public:
  /// Return the singleton instance of the CachedFileLoader.
  /// An instance will be created on first call if needed
  /// with the QApplication instance as parent.
  static CachedFileLoader* instance();

  QDir cachePath() const;

public slots:
  void setCachePath(QString cache_path);
  void load(QString url, QString cache_local_path, CachedFileClient* client);

private:
  // Make constructor/destructor private to enforce singleton pattern.
  // Only allow construction/destruction via the static instance() method.
  // The QApplication instance is a convenient parent to ensure proper
  // destruction order.
  friend class QApplication;
  CachedFileLoader(QObject* parent=nullptr);
  ~CachedFileLoader() override;

  static CachedFileLoader* instance_;

  QNetworkAccessManager* network_access_manager_;

  // Base location where files are stored locally
  QString cache_path_;

private slots:
  void downloadFinished(QNetworkReply* reply);



};

} // namespace camp

#endif
