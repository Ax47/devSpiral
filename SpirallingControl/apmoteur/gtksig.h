#ifndef _GTKSIG_H
#define _GTKSIG_H
#include <gtk/gtk.h>

void gtksig_init(); // Charge les données

void on_butStop_clicked (GtkWidget* pEntry); // Arret
void on_butInitDialClose_clicked (GtkWidget* pEntry); // Boite de dial

void on_butQuit_clicked (GtkWidget* pEntry); // Quitte appli
void on_butParams_clicked(GtkWidget* pEntry);
void on_butVelStart_active_notify(GtkWidget* pEntry);
void on_radBackward_toggled(GtkWidget* pEntry);
void on_radForward_toggled(GtkWidget* pEntry);
void on_butVelUp_clicked(GtkWidget* pEntry);
void on_butVelDown_clicked(GtkWidget* pEntry);

void on_butParamReturn_clicked(GtkWidget* pEntry);
void on_butParamSave_clicked(GtkWidget* pEntry);

void on_butDialYes_clicked(GtkWidget* pEntry);
void on_butDialNo_clicked(GtkWidget* pEntry);
#endif
