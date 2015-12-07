#ifndef _GUI_H
#define _GUI_H
#include "canfestival.h"
#include <gtk/gtk.h>

void gui_label_set(gchar* lab_id, gchar* lab_title);
void gui_label_set_objs(char* lab_title, char* lab_id, ... );
void gui_image_set(char* img_id, char* icon_name, int iconsize);
void gui_button_set(char* but_txt, char* labtxt, char* icon_name);
void gui_entry_set(gchar* ent_id, char* labtxt);
gchar* gui_entry_get(gchar* ent_id);
void gui_push_state(char * txtState);
int but_is_checked(char* labid); // Savoir si un bouton est coché
gint64 gui_str2num(gchar *txtdata); // Convertit str en double
void gui_widget2show(char* s1,...); // Affiche les widget en arg
void gui_widget2hide(char* s1,...);
void gui_info_box(char* boxTitle, char* boxContent, char* icon);
GThreadFunc gui_init();
GObject* gui_get_object(gchar* labid); // Renvoie l'objet gtk
GtkWindow* gui_get_window(gchar* labid);
GtkWidget* gui_get_widget(gchar* labid);
GtkLabel* gui_get_label(gchar* labid);
GtkButton* gui_get_button(gchar* labid);
GtkImage* gui_get_image(gchar* labid);
GtkEntry* gui_get_entry(gchar* labid);
GtkMenu* gui_get_menu(gchar* labid);
GtkDialog* gui_get_dialog(gchar* labid);
GtkSwitch* gui_get_switch(gchar* labid);
#endif
