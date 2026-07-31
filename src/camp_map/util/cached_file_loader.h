#ifndef CAMP_CACHED_FILE_LOADER_H
#define CAMP_CACHED_FILE_LOADER_H

#include <QByteArray>
#include <QDir>
#include <QObject>
#include <QString>

class QApplication;
class QNetworkAccessManager;
class QNetworkReply;

namespace camp
{

class CachedFileClient: public QObject
{
  Q_OBJECT
public:
  // expects_image opts this client's loads into CachedFileLoader's image
  // validation gate. Default false: non-tile payloads (WMTS capabilities XML)
  // load through the same shared loader and must not be gated.
  CachedFileClient(QObject* parent=nullptr, bool expects_image=false);

  bool expectsImage() const;

signals:
  void dataLoaded(QByteArray &data, CachedFileClient* client);

private:
  bool expects_image_ = false;
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

  /// Returns true if data may be cached and delivered as an image body.
  /// A declared text/XML Content-Type (a WMS ServiceExceptionReport arrives
  /// as HTTP 200 + text/xml) is rejected without a decode attempt; any other
  /// type — image/*, empty, or odd-but-valid ones like
  /// application/octet-stream — is decided by whether the bytes decode as an
  /// image.
  static bool isAcceptableImageBody(const QByteArray& data, const QString& content_type);

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
