#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <glib.h>
#include <gtk/gtk.h>
#include "strtools.h"
#include "keyword.h"

GtkBuilder *builder;


GObject* gui_get_object(gchar* labid) {
    return G_OBJECT(gtk_builder_get_object (builder, labid));
}
GtkWindow* gui_get_window(gchar* labid) {
    return GTK_WINDOW(gui_get_object(labid));
}
GtkWidget* gui_get_widget(gchar* labid) {
    return GTK_WIDGET(gui_get_object(labid));
}
GtkLabel* gui_get_label(gchar* labid) {
    return GTK_LABEL(gui_get_object(labid));
}
GtkButton* gui_get_button(gchar* labid) {
    return GTK_BUTTON(gui_get_object(labid));
}
GtkImage* gui_get_image(gchar* labid) {
    return GTK_IMAGE(gui_get_object(labid));
}
GtkEntry* gui_get_entry(gchar* labid) {
    return GTK_ENTRY(gui_get_object(labid));
}
GtkMenu* gui_get_menu(gchar* labid) {
    return GTK_MENU(gui_get_object(labid));
}
GtkDialog* gui_get_dialog(gchar* labid) {
    return GTK_DIALOG(gui_get_object(labid));
}
GtkToggleButton* gui_get_toggle(gchar* labid) {
    return GTK_TOGGLE_BUTTON(gui_get_object(labid));
}
GtkSwitch* gui_get_switch(gchar* labid) {
    return GTK_SWITCH(gui_get_object(labid));
}
void gui_label_set(gchar* lab_id, gchar* lab_title) {
    gtk_label_set_text(gui_get_label(lab_id), lab_title);
}

void gui_label_set_objs(char* lab_title, gchar* lab_id, ... ) {
    gtk_label_set_text(gui_get_label(lab_id), lab_title);
    va_list ap;
    va_start(ap,lab_id);
    gchar *arg = va_arg(ap,gchar*);
    while ( arg != NULL) {
        gtk_label_set_text(gui_get_label(arg), lab_title);
        arg = va_arg(ap,gchar*);
    }
    va_end(ap);
}

void gui_image_set(gchar* img_id, char* icon_name, int iconsize) {
    switch(iconsize) {
        case 1:
        gtk_image_set_from_icon_name (gui_get_image(img_id),icon_name,GTK_ICON_SIZE_LARGE_TOOLBAR);
        break;
        case 2:
        gtk_image_set_from_icon_name (gui_get_image(img_id),icon_name,GTK_ICON_SIZE_DND);
        break;
        case 3:
        gtk_image_set_from_icon_name (gui_get_image(img_id),icon_name,GTK_ICON_SIZE_DIALOG);
        break;
        default:
        gtk_image_set_from_icon_name (gui_get_image(img_id),icon_name,GTK_ICON_SIZE_BUTTON);
        break;
    }
}

void gui_button_set(gchar* but_id, char* labtxt, char* icon_name) {
    gtk_button_set_label (gui_get_button(but_id), labtxt);
    if (icon_name!= NULL)
        gtk_button_set_image (gui_get_button(but_id),gtk_image_new_from_icon_name (icon_name,GTK_ICON_SIZE_BUTTON));
}

void gui_entry_set(gchar* ent_id, char* labtxt) {
    gtk_entry_set_text(gui_get_entry(ent_id),labtxt);
}

const gchar* gui_entry_get(gchar* ent_id) {
    return gtk_entry_get_text(gui_get_entry(ent_id));
}


void gui_push_state(gchar* txtState) {
//    gchar* prev_txt = gtk_label_get_text (gui_get_label ("stateBarContent"));
//    int N1 = strtools_num_char_in_str(prev_txt, "\n");
//    int N2 = strtools_num_char_in_str(prev_txt, "\0");
//    if (N1 == 0 && N2 == 0) {
//        gui_label_set("stateBarContent",txtState);
//    } else {
//        if (N1 == 15) {
//            char** tab = strtools_split(prev_txt,"\n",0);
//            int i;
//            prev_txt = tab[1];
//            for (i=2; i<15;i++) {
//                prev_txt = strtools_concat(prev_txt,"\n",tab[i],NULL);
//            }
//        }
//        gui_label_set("stateBarContent",strtools_concat(txtState,"\n",prev_txt,NULL));
//    }
}

int gui_but_is_checked(char* labid) {
    return gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(gtk_builder_get_object (builder, labid)));
}

gint64 gui_str2num(gchar *txtdata) {
    return g_ascii_strtoll(txtdata,NULL,10);
}

void gui_widget2show(gchar* s1,...) {
    gtk_widget_show(gui_get_widget(s1));
    va_list ap;
    va_start(ap,s1);
    gchar *arg = va_arg(ap,gchar*);
    while ( arg != NULL) {
        gtk_widget_show(gui_get_widget(arg));
        arg = va_arg(ap,gchar*);
    }
    va_end(ap);
}

void gui_widget2hide(gchar* s1,...) {
    gtk_widget_hide(gui_get_widget(s1));
    va_list ap;
    va_start(ap,s1);
    gchar *arg = va_arg(ap,gchar*);
    while ( arg != NULL) {
        gtk_widget_hide(gui_get_widget(arg));
        arg = va_arg(ap,gchar*);
    }
    va_end(ap);
}

void gui_info_box(char* boxTitle, char* boxContent, char* icon) {
    gtk_window_set_title(gui_get_window("windowDialInit"), boxTitle);
    gui_label_set("labDialContent",boxContent);
    if (icon != NULL) {
        gui_image_set("labDialImg",icon,3);
    } else {
        gui_image_set("labDialImg","gtk-info",3);
    }
    gtk_dialog_run(gui_get_dialog("windowDialInit"));
}

gpointer gui_init(gpointer arg) {
    // Initialisation variable GTK
    GError *csserror = NULL;
    XInitThreads(); // A mettre avant gtk_init
    gtk_init (NULL, NULL);
    builder = gtk_builder_new (); // Variable globale

    // Import GLADE
    gtk_builder_add_from_file (builder, "XMLappli/Gappli_v1.glade", NULL);


    // CSS
    GFile * file = g_file_new_for_path ("./stylesheet.css");
    GtkCssProvider *cssprovider = gtk_css_provider_new ();
    if (!gtk_css_provider_load_from_file (cssprovider, file, &csserror)) {
        g_printerr ("%s\n", csserror->message);
        exit (csserror->code);
    }
    gtk_style_context_add_provider_for_screen (gdk_display_get_default_screen (gtk_widget_get_display (gui_get_widget("mainWindow"))), GTK_STYLE_PROVIDER (cssprovider), GTK_STYLE_PROVIDER_PRIORITY_USER);

    // Init
    gtksig_init();
    keyword_init();
    gtk_widget_show_all (gui_get_widget("mainWindow"));
    //gtk_widget_set_visible(gui_get_widget("but_LaserSimuStart"),0);


    gtk_main();
    return 0;
}
